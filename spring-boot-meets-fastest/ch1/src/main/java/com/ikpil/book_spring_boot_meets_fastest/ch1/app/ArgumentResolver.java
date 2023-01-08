package com.ikpil.book_spring_boot_meets_fastest.ch1.app;

import java.io.InputStream;

public interface ArgumentResolver {
    Argument resolve(InputStream stream);
}
