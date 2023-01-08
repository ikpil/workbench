package com.ikpil.book_spring_boot_meets_fastest.ch5.service;

import com.ikpil.book_spring_boot_meets_fastest.ch5.repository.UserRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

@Service // 컴포넌트 스캔 되도록 추가
public class LoginUserDetailsService implements UserDetailsService {
    @Autowired
    UserRepository userRepository; // 유저 객체에 접근하기 위해 주입

    @Override
    public UserDetails loadUserByUsername(String username) throws UsernameNotFoundException {
        var user = userRepository.findById(username)
                .orElseThrow(() -> new UsernameNotFoundException("The requested user is not found."));

        return new LoginUserDetails(user);
    }
}
