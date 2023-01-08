package com.ikpil.hello.java.core;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.BooleanSupplier;

public class Log4J2Example implements Example {
    private static final Logger logger = LoggerFactory.getLogger(CollectionExample.class);

    public void run() {
        logger.trace("logger trace");
        logger.debug("logger debug");
        logger.info("logger info");
        logger.warn("logger warn");
        logger.error("logger error");

        logger.info("logger info - var1({}) var2({}) var3({})", 1, 1.f, "asdf");

        int case1 = 1;
        lazyLog(() -> logger.info("lazyLog by lambda - var({})", case1));

        int case2 = 2;
        lazyLog(false, () -> logger.info("lazyLog by lambda - var({})", case2));

        int case3 = 3;
        lazyLog(true, () -> logger.info("lazyLog by lambda - var({})", case3));

        int case4 = 4;
        lazyLog(() -> true, () -> logger.info("lazyLog by lambda - var({})", case4));
    }

    private void lazyLog(Runnable event) {
        lazyLog(true, event);
    }

    private void lazyLog(BooleanSupplier f, Runnable event) {
        lazyLog(f.getAsBoolean(), event);
    }

    private void lazyLog(boolean c, Runnable event) {
        if (!c)
            return;

        event.run();
    }
}
