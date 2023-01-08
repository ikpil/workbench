package com.ikpil.book_spring_boot_meets_fastest.ch2.repository;

import com.ikpil.book_spring_boot_meets_fastest.ch2.domain.Customer;
import org.springframework.data.jpa.repository.JpaRepository;

public interface CustomerRepository extends JpaRepository<Customer, Integer> {

}

