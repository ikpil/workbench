package com.ikpil.book_spring_boot_meets_fastest.ch4;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.authentication.builders.AuthenticationManagerBuilder;
import org.springframework.security.config.annotation.authentication.configuration.GlobalAuthenticationConfigurerAdapter;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.builders.WebSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.annotation.web.configuration.WebSecurityConfigurerAdapter;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.crypto.bcrypt.BCryptPasswordEncoder;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.security.web.util.matcher.AntPathRequestMatcher;

@EnableWebSecurity // 스프링 시큐리티에 관련된 기본적인 사항들이 설정된다
@Configuration
public class SecurityConfig extends WebSecurityConfigurerAdapter {
    @Override
    public void configure(WebSecurity web) throws Exception { // 특정 요청에 대해서 시큐리티 설정 무시
        web.ignoring().antMatchers("/webjars/**", "/css/**"); // 정적 리소스 URL 지정하여 무시
    }

    @Override
    protected void configure(HttpSecurity http) throws Exception { // 오버라이드 하여, 설정을 한다
//        // security.basic.enabled: false 해당 옵션이 없어졌기 때문에, 일단 코드로 구현해야 한다
//        // https://stackoverflow.com/questions/49717573/property-security-basic-enabled-is-deprecated-the-security-auto-configuration
//        http.authorizeRequests().anyRequest().permitAll();

        // 인가 설정
        http.authorizeRequests()
                .antMatchers("/loginForm").permitAll() // /loginForm 만 모든 유저 접속 인가
                .anyRequest()
                .authenticated();

        // 로그인 설정
        http.formLogin()
                .loginProcessingUrl("/login")
                .loginPage("/loginForm")
                .failureUrl("/loginForm?error")
                .defaultSuccessUrl("/customers", true)
                .usernameParameter("username")
                .passwordParameter("password")
                .and();

        // 로그아웃 설정
        http.logout()
                .logoutRequestMatcher(new AntPathRequestMatcher("/logout**"))
                .logoutSuccessUrl("/loginForm");
    }

    // 인증 처리에 관련된 사항을 설정한다.
    @Configuration
    static class AuthenticationConfiguration extends GlobalAuthenticationConfigurerAdapter {
        @Autowired
        UserDetailsService userDetailsService;

        // 암호를 해쉬 형태로 받기 위해서 PasswordEncoder 정의
        @Bean
        PasswordEncoder passwordEncoder() {
            return new BCryptPasswordEncoder();
        }

        @Override
        public void init(AuthenticationManagerBuilder auth) throws Exception {
            auth.userDetailsService(userDetailsService)
                    .passwordEncoder(passwordEncoder());
        }
    }
}