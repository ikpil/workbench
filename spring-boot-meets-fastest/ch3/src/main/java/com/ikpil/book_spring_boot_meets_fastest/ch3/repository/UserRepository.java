package com.ikpil.book_spring_boot_meets_fastest.ch3.repository;

import com.ikpil.book_spring_boot_meets_fastest.ch3.domain.User;
import org.springframework.data.jpa.repository.JpaRepository;

public interface UserRepository extends JpaRepository<User, String> {
}
