package com.ikpil.book_spring_boot_meets_fastest.ch1;

import com.ikpil.book_spring_boot_meets_fastest.ch1.app.Argument;
import com.ikpil.book_spring_boot_meets_fastest.ch1.app.ArgumentResolver;
import com.ikpil.book_spring_boot_meets_fastest.ch1.app.Calculator;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.annotation.ComponentScan;

@EnableAutoConfiguration
@ComponentScan // 이 클래스의 패키지 내부에 있는 모든 클래스의 Component 를 검색한다
public class App implements CommandLineRunner {
    @Autowired
    ArgumentResolver argumentResolver;
    @Autowired
    Calculator calculator;

    @Override
    public void run(String... strings) {
        System.out.print("Enter 2 numbers like `a b` : ");
        Argument argument = argumentResolver.resolve(System.in);

        int result = calculator.calc(argument.getA(), argument.getB());
        System.out.println("result = " + result);

    }

    public static void main(String[] args) {
        SpringApplication.run(App.class, args);
    }
}
