package com.ikpil.book_spring_boot_meets_fastest.ch1.app;

import org.springframework.stereotype.Component;

import java.io.InputStream;
import java.util.Scanner;

@Component
public class ScannerArgumentResolver implements ArgumentResolver {
    @Override
    public Argument resolve(InputStream stream) {
        var scanner = new Scanner(stream);

        int a = 0;
        if (scanner.hasNext())
            a = scanner.nextInt();

        int b = 0;
        if (scanner.hasNext())
            b = scanner.nextInt();

        return new Argument(a, b);
    }
}
