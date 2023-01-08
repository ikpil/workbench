package com.ikpil.book_spring_boot_meets_fastest.ch5.test;

import com.ikpil.book_spring_boot_meets_fastest.ch5.App;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.boot.web.server.LocalServerPort;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.test.context.junit4.SpringJUnit4ClassRunner;


@RunWith(SpringJUnit4ClassRunner.class)
@SpringBootTest(classes = App.class, webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
public class AppTest {
    @LocalServerPort
    int port;

    @Autowired
    TestRestTemplate restTemplate;

    @Test
    public void testHome() {
        ResponseEntity<String> response = restTemplate.getForEntity("http://localhost:" + port, String.class);

        Assert.assertThat(response.getStatusCode(), CoreMatchers.is(HttpStatus.OK));
        Assert.assertThat(response.getBody(), CoreMatchers.is("Hello World!"));
    }
}
