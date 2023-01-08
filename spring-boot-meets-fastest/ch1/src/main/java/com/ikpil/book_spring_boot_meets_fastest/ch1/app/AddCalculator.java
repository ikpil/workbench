package com.ikpil.book_spring_boot_meets_fastest.ch1.app;

import org.springframework.stereotype.Component;

@Component
public class AddCalculator implements Calculator {
    @Override
    public int calc(int a, int b) {
        return a + b;
    }
}
