package com.ikpil.book_spring_boot_meets_fastest.ch1.app;

import lombok.Data;
import lombok.RequiredArgsConstructor;

@Data
@RequiredArgsConstructor
public class Argument {
    private final int a;
    private final int b;
}
