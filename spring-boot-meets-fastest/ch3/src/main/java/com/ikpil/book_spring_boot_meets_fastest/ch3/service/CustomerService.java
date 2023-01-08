package com.ikpil.book_spring_boot_meets_fastest.ch3.service;

import com.ikpil.book_spring_boot_meets_fastest.ch3.domain.Customer;
import com.ikpil.book_spring_boot_meets_fastest.ch3.domain.User;
import com.ikpil.book_spring_boot_meets_fastest.ch3.repository.CustomerRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

@Transactional
@Service
public class CustomerService {
    @Autowired
    CustomerRepository repository;

    public List<Customer> findAll() {
        //return repository.findAllOrderByName();
        return repository.findAllWithUserOrderByName();

    }

    public Page<Customer> findAll(Pageable pageable) {
        return repository.findAll(pageable);
    }

    public Customer findOne(Integer id) {
        return repository.findById(id).orElse(null);
    }

    public Customer create(Customer customer, User user) {
        customer.setUser(user);
        return repository.save(customer);
    }

    public Customer update(Customer customer) {
        return repository.save(customer);
    }

    public void delete(Integer id) {
        repository.deleteById(id);
    }

}
