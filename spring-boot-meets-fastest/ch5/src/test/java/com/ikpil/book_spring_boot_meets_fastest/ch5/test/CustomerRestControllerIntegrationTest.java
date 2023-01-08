package com.ikpil.book_spring_boot_meets_fastest.ch5.test;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.ikpil.book_spring_boot_meets_fastest.ch5.App;
import com.ikpil.book_spring_boot_meets_fastest.ch5.domain.Customer;
import com.ikpil.book_spring_boot_meets_fastest.ch5.domain.User;
import com.ikpil.book_spring_boot_meets_fastest.ch5.repository.CustomerRepository;
import com.ikpil.book_spring_boot_meets_fastest.ch5.repository.UserRepository;
import lombok.Data;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.boot.web.server.LocalServerPort;
import org.springframework.core.ParameterizedTypeReference;
import org.springframework.http.HttpMethod;
import org.springframework.http.HttpStatus;
import org.springframework.test.context.junit4.SpringJUnit4ClassRunner;

import java.util.List;

@RunWith(SpringJUnit4ClassRunner.class)
@SpringBootTest(
        classes = App.class,
        webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT,
        properties = {
                "spring.datasource.url:jdbc:h2:mem:bookmark:DB_CLOSE_ON_EXIT=FALSE"
        }
)
public class CustomerRestControllerIntegrationTest {
    @LocalServerPort
    int port;

    @Autowired
    CustomerRepository customerRepository;

    @Autowired
    UserRepository userRepository;


    String apiEndpoint;

    @Autowired
    TestRestTemplate restTemplate;

    Customer customer1;
    Customer customer2;

    @Before
    public void setUp() {
        customerRepository.deleteAll();
        userRepository.deleteAll();

        var user = new User("admin", "babo", null);
        userRepository.save(user);

        customer1 = new Customer();
        customer1.setFirstName("마이글");
        customer1.setLastName("페러데이");
        customer1.setUser(user);
        customerRepository.save(customer1);

        customer2 = new Customer();
        customer2.setFirstName("아이작");
        customer2.setLastName("뉴턴");
        customer2.setUser(user);
        customerRepository.save(customer2);

        apiEndpoint = "http://localhost:" + port + "/api/customers";
    }

    @Test
    public void testGetCustomers() throws Exception {
        var response = restTemplate.exchange(apiEndpoint, HttpMethod.GET, null,
                new ParameterizedTypeReference<Page<Customer>>() {
                });

        Assert.assertThat(response.getStatusCode(), CoreMatchers.is(HttpStatus.OK));
        Assert.assertThat(response.getBody().numberOfElements, CoreMatchers.is(2));
    }

    @Data
    @JsonIgnoreProperties(ignoreUnknown = true)
    static class Page<T> {
        private List<T> content;
        private int numberOfElements;
    }


}
