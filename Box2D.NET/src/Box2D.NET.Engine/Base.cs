// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine
{
    /// Version numbering scheme.
    /// See https://semver.org/
    public struct b2Version
    {
        /// Significant changes
        int major;

        /// Incremental changes
        int minor;

        /// Bug fixes
        int revision;
    }
}