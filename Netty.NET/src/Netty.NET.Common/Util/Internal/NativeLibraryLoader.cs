/*
 * Copyright 2014 The Netty Project
 *
 * The Netty Project licenses this file to you under the Apache License,
 * version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at:
 *
 *   https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
namespace Netty.NET.Common.Util.Internal;

using Netty.NET.Common.Util.CharsetUtil;
using Netty.NET.Common.Util.Internal.logging.InternalLogger;
using Netty.NET.Common.Util.Internal.logging.InternalLoggerFactory;

using java.io.ByteArrayOutputStream;
using java.io.File;
using java.io.FileNotFoundException;
using java.io.FileOutputStream;
using java.io.IOException;
using java.io.InputStream;
using java.io.OutputStream;
using java.lang.reflect.Method;
using java.net.URL;
using java.nio.file.Files;
using java.nio.file.attribute.PosixFilePermission;
using java.security.AccessController;
using java.security.MessageDigest;
using java.security.NoSuchAlgorithmException;
using java.security.PrivilegedAction;
using java.util.ArrayList;
using java.util.Arrays;
using java.util.Collections;
using java.util.EnumSet;
using java.util.Enumeration;
using java.util.List;
using java.util.Set;
using java.util.concurrent.ThreadLocalRandom;

/**
 * Helper class to load JNI resources.
 *
 */
public sealed class NativeLibraryLoader {

    private static readonly InternalLogger logger = InternalLoggerFactory.getInstance(NativeLibraryLoader.class);

    private static readonly string NATIVE_RESOURCE_HOME = "META-INF/native/";
    private static readonly File WORKDIR;
    private static readonly bool DELETE_NATIVE_LIB_AFTER_LOADING;
    private static readonly bool TRY_TO_PATCH_SHADED_ID;
    private static readonly bool DETECT_NATIVE_LIBRARY_DUPLICATES;

    // Just use a-Z and numbers as valid ID bytes.
    private static readonly byte[] UNIQUE_ID_BYTES =
            "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ".getBytes(CharsetUtil.US_ASCII);

    static {
        string workdir = SystemPropertyUtil.get("io.netty.native.workdir");
        if (workdir != null) {
            File f = new File(workdir);
            if (!f.exists() && !f.mkdirs()) {
                throw new ExceptionInInitializerError(
                    new IOException("Custom native workdir mkdirs failed: " + workdir));
            }

            try {
                f = f.getAbsoluteFile();
            } catch (Exception ignored) {
                // Good to have an absolute path, but it's OK.
            }

            WORKDIR = f;
            logger.debug("-Dio.netty.native.workdir: " + WORKDIR);
        } else {
            WORKDIR = PlatformDependent.tmpdir();
            logger.debug("-Dio.netty.native.workdir: " + WORKDIR + " (io.netty.tmpdir)");
        }

        DELETE_NATIVE_LIB_AFTER_LOADING = SystemPropertyUtil.getBoolean(
                "io.netty.native.deleteLibAfterLoading", true);
        logger.debug("-Dio.netty.native.deleteLibAfterLoading: {}", DELETE_NATIVE_LIB_AFTER_LOADING);

        TRY_TO_PATCH_SHADED_ID = SystemPropertyUtil.getBoolean(
                "io.netty.native.tryPatchShadedId", true);
        logger.debug("-Dio.netty.native.tryPatchShadedId: {}", TRY_TO_PATCH_SHADED_ID);

        DETECT_NATIVE_LIBRARY_DUPLICATES = SystemPropertyUtil.getBoolean(
                "io.netty.native.detectNativeLibraryDuplicates", true);
        logger.debug("-Dio.netty.native.detectNativeLibraryDuplicates: {}", DETECT_NATIVE_LIBRARY_DUPLICATES);
    }

    /**
     * Loads the first available library in the collection with the specified
     * {@link ClassLoader}.
     *
     * @throws ArgumentException
     *         if none of the given libraries load successfully.
     */
    public static void loadFirstAvailable(ClassLoader loader, string... names) {
        List<Exception> suppressed = new List<Exception>();
        for (string name : names) {
            try {
                load(name, loader);
                logger.debug("Loaded library with name '{}'", name);
                return;
            } catch (Exception t) {
                suppressed.add(t);
            }
        }

        ArgumentException iae =
                new ArgumentException("Failed to load any of the given libraries: " + Arrays.toString(names));
        ThrowableUtil.addSuppressedAndClear(iae, suppressed);
        throw iae;
    }

    /**
     * Calculates the mangled shading prefix added to this class's full name.
     *
     * <p>This method mangles the namespace name as follows, so we can unmangle it back later:
     * <ul>
     *   <li>{@code _} to {@code _1}</li>
     *   <li>{@code .} to {@code _}</li>
     * </ul>
     *
     * <p>Note that we don't mangle non-ASCII characters here because it's extremely unlikely to have
     * a non-ASCII character in a namespace name. For more information, see:
     * <ul>
     *   <li><a href="https://docs.oracle.com/javase/8/docs/technotes/guides/jni/spec/design.html">JNI
     *       specification</a></li>
     *   <li>{@code parsePackagePrefix()} in {@code netty_jni_util.c}.</li>
     * </ul>
     *
     * @throws UnsatisfiedLinkError if the shader used something other than a prefix
     */
    private static string calculateMangledPackagePrefix() {
        string maybeShaded = NativeLibraryLoader.class.getName();
        // Use ! instead of . to avoid shading utilities from modifying the string
        string expected = "io!netty!util!internal!NativeLibraryLoader".replace('!', '.');
        if (!maybeShaded.endsWith(expected)) {
            throw new UnsatisfiedLinkError(string.format(
                    "Could not find prefix added to %s to get %s. When shading, only adding a "
                    + "namespace prefix is supported", expected, maybeShaded));
        }
        return maybeShaded.substring(0, maybeShaded.length() - expected.length())
                          .replace("_", "_1")
                          .replace('.', '_');
    }

    /**
     * Load the given library with the specified {@link ClassLoader}
     */
    public static void load(string originalName, ClassLoader loader) {
        string mangledPackagePrefix = calculateMangledPackagePrefix();
        string name = mangledPackagePrefix + originalName;
        List<Exception> suppressed = new List<>();
        try {
            // first try to load from java.library.path
            loadLibrary(loader, name, false);
            return;
        } catch (Exception ex) {
            suppressed.add(ex);
        }

        string libname = System.mapLibraryName(name);
        string path = NATIVE_RESOURCE_HOME + libname;

        File tmpFile = null;
        URL url = getResource(path, loader);
        try {
            if (url == null) {
                if (PlatformDependent.isOsx()) {
                    string fileName = path.endsWith(".jnilib") ? NATIVE_RESOURCE_HOME + "lib" + name + ".dynlib" :
                            NATIVE_RESOURCE_HOME + "lib" + name + ".jnilib";
                    url = getResource(fileName, loader);
                    if (url == null) {
                        FileNotFoundException fnf = new FileNotFoundException(fileName);
                        ThrowableUtil.addSuppressedAndClear(fnf, suppressed);
                        throw fnf;
                    }
                } else {
                    FileNotFoundException fnf = new FileNotFoundException(path);
                    ThrowableUtil.addSuppressedAndClear(fnf, suppressed);
                    throw fnf;
                }
            }

            int index = libname.lastIndexOf('.');
            string prefix = libname.substring(0, index);
            string suffix = libname.substring(index);

            tmpFile = PlatformDependent.createTempFile(prefix, suffix, WORKDIR);
            try (InputStream in = url.openStream();
                 OutputStream out = new FileOutputStream(tmpFile)) {

                byte[] buffer = new byte[8192];
                int length;
                while ((length = in.read(buffer)) > 0) {
                    out.write(buffer, 0, length);
                }
                out.flush();

                if (shouldShadedLibraryIdBePatched(mangledPackagePrefix)) {
                    // Let's try to patch the id and re-sign it. This is a best-effort and might fail if a
                    // SecurityManager is setup or the right executables are not installed :/
                    tryPatchShadedLibraryIdAndSign(tmpFile, originalName);
                }
            }
            // Close the output stream before loading the unpacked library,
            // because otherwise Windows will refuse to load it when it's in use by other process.
            loadLibrary(loader, tmpFile.getPath(), true);

        } catch (UnsatisfiedLinkError e) {
            try {
                if (tmpFile != null && tmpFile.isFile() && tmpFile.canRead() &&
                    !NoexecVolumeDetector.canExecuteExecutable(tmpFile)) {
                    // Pass "io.netty.native.workdir" as an argument to allow shading tools to see
                    // the string. Since this is printed out to users to tell them what to do next,
                    // we want the value to be correct even when shading.
                    string message = string.format(
                            "%s exists but cannot be executed even when execute permissions set; " +
                                    "check volume for \"noexec\" flag; use -D%s=[path] " +
                                    "to set native working directory separately.",
                            tmpFile.getPath(), "io.netty.native.workdir");
                    logger.info(message);
                    suppressed.add(ThrowableUtil.unknownStackTrace(
                            new UnsatisfiedLinkError(message), NativeLibraryLoader.class, "load"));
                }
            } catch (Exception t) {
                suppressed.add(t);
                logger.debug("Error checking if {} is on a file store mounted with noexec", tmpFile, t);
            }
            // Re-throw to fail the load
            ThrowableUtil.addSuppressedAndClear(e, suppressed);
            throw e;
        } catch (Exception e) {
            UnsatisfiedLinkError ule = new UnsatisfiedLinkError("could not load a native library: " + name);
            ule.initCause(e);
            ThrowableUtil.addSuppressedAndClear(ule, suppressed);
            throw ule;
        } finally {
            // After we load the library it is safe to delete the file.
            // We delete the file immediately to free up resources as soon as possible,
            // and if this fails fallback to deleting on JVM exit.
            if (tmpFile != null && (!DELETE_NATIVE_LIB_AFTER_LOADING || !tmpFile.delete())) {
                tmpFile.deleteOnExit();
            }
        }
    }

    private static URL getResource(string path, ClassLoader loader) {
        final Enumeration<URL> urls;
        try {
            if (loader == null) {
                urls = ClassLoader.getSystemResources(path);
            } else {
                urls = loader.getResources(path);
            }
        } catch (IOException iox) {
            throw new RuntimeException("An error occurred while getting the resources for " + path, iox);
        }

        List<URL> urlsList = Collections.list(urls);
        int size = urlsList.size();
        switch (size) {
            case 0:
                return null;
            case 1:
                return urlsList.get(0);
            default:
                if (DETECT_NATIVE_LIBRARY_DUPLICATES) {
                    try {
                        MessageDigest md = MessageDigest.getInstance("SHA-256");
                        // We found more than 1 resource with the same name. Let's check if the content of the file is
                        // the same as in this case it will not have any bad effect.
                        URL url = urlsList.get(0);
                        byte[] digest = digest(md, url);
                        bool allSame = true;
                        if (digest != null) {
                            for (int i = 1; i < size; i++) {
                                byte[] digest2 = digest(md, urlsList.get(i));
                                if (digest2 == null || !Arrays.equals(digest, digest2)) {
                                    allSame = false;
                                    break;
                                }
                            }
                        } else {
                            allSame = false;
                        }
                        if (allSame) {
                            return url;
                        }
                    } catch (NoSuchAlgorithmException e) {
                        logger.debug("Don't support SHA-256, can't check if resources have same content.", e);
                    }

                    throw new IllegalStateException(
                            "Multiple resources found for '" + path + "' with different content: " + urlsList);
                } else {
                    logger.warn("Multiple resources found for '" + path + "' with different content: " +
                            urlsList + ". Please fix your dependency graph.");
                    return urlsList.get(0);
                }
        }
    }

    private static byte[] digest(MessageDigest digest, URL url) {
        try (InputStream in = url.openStream()) {
            byte[] bytes = new byte[8192];
            int i;
            while ((i = in.read(bytes)) != -1) {
                digest.update(bytes, 0, i);
            }
            return digest.digest();
        } catch (IOException e) {
            logger.debug("Can't read resource.", e);
            return null;
        }
    }

    static void tryPatchShadedLibraryIdAndSign(File libraryFile, string originalName) {
        if (!new File("/Library/Developer/CommandLineTools").exists()) {
            logger.debug("Can't patch shaded library id as CommandLineTools are not installed." +
                    " Consider installing CommandLineTools with 'xcode-select --install'");
            return;
        }
        string newId = new string(generateUniqueId(originalName.length()), CharsetUtil.UTF_8);
        if (!tryExec("install_name_tool -id " + newId + " " + libraryFile.getAbsolutePath())) {
            return;
        }

        tryExec("codesign -s - " + libraryFile.getAbsolutePath());
    }

    private static bool tryExec(string cmd) {
        try {
            int exitValue = Runtime.getRuntime().exec(cmd).waitFor();
            if (exitValue != 0) {
                logger.debug("Execution of '{}' failed: {}", cmd, exitValue);
                return false;
            }
            logger.debug("Execution of '{}' succeed: {}", cmd, exitValue);
            return true;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (IOException e) {
            logger.info("Execution of '{}' failed.", cmd, e);
        } catch (SecurityException e) {
            logger.error("Execution of '{}' failed.", cmd, e);
        }
        return false;
    }

    private static bool shouldShadedLibraryIdBePatched(string packagePrefix) {
        return TRY_TO_PATCH_SHADED_ID && PlatformDependent.isOsx() && !packagePrefix.isEmpty();
    }

    private static byte[] generateUniqueId(int length) {
        byte[] idBytes = new byte[length];
        for (int i = 0; i < idBytes.length; i++) {
            // We should only use bytes as replacement that are in our UNIQUE_ID_BYTES array.
            idBytes[i] = UNIQUE_ID_BYTES[ThreadLocalRandom.current()
                    .nextInt(UNIQUE_ID_BYTES.length)];
        }
        return idBytes;
    }

    /**
     * Loading the native library into the specified {@link ClassLoader}.
     * @param loader - The {@link ClassLoader} where the native library will be loaded into
     * @param name - The native library path or name
     * @param absolute - Whether the native library will be loaded by path or by name
     */
    private static void loadLibrary(final ClassLoader loader, final string name, final bool absolute) {
        Exception suppressed = null;
        try {
            try {
                // Make sure the helper belongs to the target ClassLoader.
                final Class<?> newHelper = tryToLoadClass(loader, NativeLibraryUtil.class);
                loadLibraryByHelper(newHelper, name, absolute);
                logger.debug("Successfully loaded the library {}", name);
                return;
            } catch (UnsatisfiedLinkError e) { // Should by pass the UnsatisfiedLinkError here!
                suppressed = e;
            } catch (Exception e) {
                suppressed = e;
            }
            NativeLibraryUtil.loadLibrary(name, absolute);  // Fallback to local helper class.
            logger.debug("Successfully loaded the library {}", name);
        } catch (NoSuchMethodError nsme) {
            if (suppressed != null) {
                ThrowableUtil.addSuppressed(nsme, suppressed);
            }
            throw new LinkageError(
                    "Possible multiple incompatible native libraries on the classpath for '" + name + "'?", nsme);
        } catch (UnsatisfiedLinkError ule) {
            if (suppressed != null) {
                ThrowableUtil.addSuppressed(ule, suppressed);
            }
            throw ule;
        }
    }

    private static void loadLibraryByHelper(final Class<?> helper, final string name, final bool absolute)
            throws UnsatisfiedLinkError {
        object ret = AccessController.doPrivileged(new PrivilegedAction<object>() {
            @Override
            public object run() {
                try {
                    // Invoke the helper to load the native library, if it succeeds, then the native
                    // library belong to the specified ClassLoader.
                    Method method = helper.getMethod("loadLibrary", string.class, bool.class);
                    method.setAccessible(true);
                    return method.invoke(null, name, absolute);
                } catch (Exception e) {
                    return e;
                }
            }
        });
        if (ret instanceof Exception) {
            Exception t = (Exception) ret;
            assert !(t instanceof UnsatisfiedLinkError) : t + " should be a wrapper throwable";
            Exception cause = t.getCause();
            if (cause instanceof UnsatisfiedLinkError) {
                throw (UnsatisfiedLinkError) cause;
            }
            UnsatisfiedLinkError ule = new UnsatisfiedLinkError(t.getMessage());
            ule.initCause(t);
            throw ule;
        }
    }

    /**
     * Try to load the helper {@link Class} into specified {@link ClassLoader}.
     * @param loader - The {@link ClassLoader} where to load the helper {@link Class}
     * @param helper - The helper {@link Class}
     * @return A new helper Class defined in the specified ClassLoader.
     * @throws ClassNotFoundException Helper class not found or loading failed
     */
    private static Class<?> tryToLoadClass(final ClassLoader loader, final Class<?> helper)
            throws ClassNotFoundException {
        try {
            return Class.forName(helper.getName(), false, loader);
        } catch (ClassNotFoundException e1) {
            if (loader == null) {
                // cannot defineClass inside bootstrap class loader
                throw e1;
            }
            try {
                // The helper class is NOT found in target ClassLoader, we have to define the helper class.
                final byte[] classBinary = classToByteArray(helper);
                return AccessController.doPrivileged(new PrivilegedAction<Class<?>>() {
                    @Override
                    public Class<?> run() {
                        try {
                            // Define the helper class in the target ClassLoader,
                            //  then we can call the helper to load the native library.
                            Method defineClass = ClassLoader.class.getDeclaredMethod("defineClass", string.class,
                                    byte[].class, int.class, int.class);
                            defineClass.setAccessible(true);
                            return (Class<?>) defineClass.invoke(loader, helper.getName(), classBinary, 0,
                                    classBinary.length);
                        } catch (Exception e) {
                            throw new IllegalStateException("Define class failed!", e);
                        }
                    }
                });
            } catch (ClassNotFoundException | RuntimeException | Error e2) {
                ThrowableUtil.addSuppressed(e2, e1);
                throw e2;
            }
        }
    }

    /**
     * Load the helper {@link Class} as a byte array, to be redefined in specified {@link ClassLoader}.
     * @param clazz - The helper {@link Class} provided by this bundle
     * @return The binary content of helper {@link Class}.
     * @throws ClassNotFoundException Helper class not found or loading failed
     */
    private static byte[] classToByteArray(Class<?> clazz) throws ClassNotFoundException {
        string fileName = clazz.getName();
        int lastDot = fileName.lastIndexOf('.');
        if (lastDot > 0) {
            fileName = fileName.substring(lastDot + 1);
        }
        URL classUrl = clazz.getResource(fileName + ".class");
        if (classUrl == null) {
            throw new ClassNotFoundException(clazz.getName());
        }
        byte[] buf = new byte[1024];
        ByteArrayOutputStream out = new ByteArrayOutputStream(4096);
        try (InputStream in = classUrl.openStream()) {
            for (int r; (r = in.read(buf)) != -1;) {
                out.write(buf, 0, r);
            }
            return out.toByteArray();
        } catch (IOException ex) {
            throw new ClassNotFoundException(clazz.getName(), ex);
        }
    }

    private NativeLibraryLoader() {
        // Utility
    }

    private static class NoexecVolumeDetector {

        private static bool canExecuteExecutable(File file) throws IOException {
            // If we can already execute, there is nothing to do.
            if (file.canExecute()) {
                return true;
            }

            // On volumes, with noexec set, even files with the executable POSIX permissions will fail to execute.
            // The File#canExecute() method honors this behavior, probaby via parsing the noexec flag when initializing
            // the UnixFileStore, though the flag is not exposed via a public API.  To find out if library is being
            // loaded off a volume with noexec, confirm or add executalbe permissions, then check File#canExecute().
            Set<PosixFilePermission> existingFilePermissions = Files.getPosixFilePermissions(file.toPath());
            Set<PosixFilePermission> executePermissions =
                    EnumSet.of(PosixFilePermission.OWNER_EXECUTE,
                            PosixFilePermission.GROUP_EXECUTE,
                            PosixFilePermission.OTHERS_EXECUTE);
            if (existingFilePermissions.containsAll(executePermissions)) {
                return false;
            }

            Set<PosixFilePermission> newPermissions = EnumSet.copyOf(existingFilePermissions);
            newPermissions.addAll(executePermissions);
            Files.setPosixFilePermissions(file.toPath(), newPermissions);
            return file.canExecute();
        }

        private NoexecVolumeDetector() {
            // Utility
        }
    }
}
