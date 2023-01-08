package com.ikpil.book_spring_boot_meets_fastest.ch5.service;

import com.ikpil.book_spring_boot_meets_fastest.ch5.domain.User;
import org.springframework.security.core.authority.AuthorityUtils;

public class LoginUserDetails extends org.springframework.security.core.userdetails.User {
    public final User user;

    public LoginUserDetails(User user) {
        super(user.getUsername(), user.getEncodedPassword(), AuthorityUtils.createAuthorityList("ROLE_USER"));
        this.user = user;
    }
}
