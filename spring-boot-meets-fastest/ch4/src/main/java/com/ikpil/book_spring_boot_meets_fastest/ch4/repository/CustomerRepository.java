package com.ikpil.book_spring_boot_meets_fastest.ch4.repository;

import com.ikpil.book_spring_boot_meets_fastest.ch4.domain.Customer;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;

public interface CustomerRepository extends JpaRepository<Customer, Integer> {
    //@Query("SELECT x FROM Customer x ORDER BY x.firstName, x.lastName")
    @Query("SELECT x FROM Customer x ORDER BY x.id DESC")
    List<Customer> findAllOrderByName();

    // N+1 SELECT 문제 해결
    @Query("SELECT DISTINCT x FROM Customer x JOIN FETCH x.user ORDER BY x.firstName, x.lastName")
    List<Customer> findAllWithUserOrderByName();
}

