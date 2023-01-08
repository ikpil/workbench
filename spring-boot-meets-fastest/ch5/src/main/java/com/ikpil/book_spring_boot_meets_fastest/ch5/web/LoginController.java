package com.ikpil.book_spring_boot_meets_fastest.ch5.web;

import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestMapping;

@Controller
public class LoginController {
    @RequestMapping("loginForm")
    String loginForm() {
        return "loginForm";
    }
}
