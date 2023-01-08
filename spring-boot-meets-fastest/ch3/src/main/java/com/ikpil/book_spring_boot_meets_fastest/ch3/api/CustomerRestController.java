package com.ikpil.book_spring_boot_meets_fastest.ch3.api;

import com.ikpil.book_spring_boot_meets_fastest.ch3.domain.Customer;
import com.ikpil.book_spring_boot_meets_fastest.ch3.service.CustomerService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.web.PageableDefault;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

@RestController // REST 웹 서비스의 엔드 포인트 컨트롤러 클래스에 지정하는 애너테이션
@RequestMapping("api/customers")  // REST 웹 서비스의 URL과 매핑 애너테이션
public class CustomerRestController {
    @Autowired
    CustomerService customerService;

    // REST API 중 GET 과 매핑
    @RequestMapping(method = RequestMethod.GET)
    Page<Customer> getCustomers(@PageableDefault Pageable pageable) {
        var customers = customerService.findAll(pageable);
        return customers;
    }

    // REST API 중 ../{id} 로 매핑
    @RequestMapping(value = "{id}", method = RequestMethod.GET)
    Customer getCustomer(@PathVariable Integer id) {
        return customerService.findOne(id);
    }

    @RequestMapping(method = RequestMethod.POST)
    @ResponseStatus(HttpStatus.CREATED)
    Customer postCustomers(@RequestBody Customer customer) {
        //return customerService.create(customer);
        return null;
    }

    @RequestMapping(value = "{id}", method = RequestMethod.PUT)
    Customer postCustomer(@PathVariable Integer id, @RequestBody Customer customer) {
        customer.setId(id);
        return customerService.update(customer);
    }

    @RequestMapping(value = "{id}", method = RequestMethod.DELETE)
    @ResponseStatus(HttpStatus.NO_CONTENT)
    void postCustomer(@PathVariable Integer id) {
        customerService.delete(id);
    }
}
