package com.ikpil.book_spring_boot_meets_fastest.ch2;

import com.ikpil.book_spring_boot_meets_fastest.ch2.domain.Customer;
import com.ikpil.book_spring_boot_meets_fastest.ch2.repository.CustomerRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.data.domain.PageRequest;

@SpringBootApplication
@ComponentScan
public class App implements CommandLineRunner {
    @Autowired
    CustomerRepository repository;

    public static void main(String[] args) {
        SpringApplication.run(App.class, args);
    }

    @Override
    public void run(String... strings) throws Exception {
        var created = repository.save(new Customer(null, "밀란", "쿤데라"));
        System.out.println(created + " is created!");

        var pageable = PageRequest.of(0, 3);
        var page = repository.findAll(pageable);
        System.out.println("한 페이지당 데이터 수 = " + page.getSize());
        System.out.println("현재 페이지 = " + page.getNumber());
        System.out.println("전체 페이지 수 = " + page.getTotalPages());
        System.out.println("전체 데이터 수 = " + page.getTotalElements());

        page.getContent().forEach(System.out::println);
    }
}
