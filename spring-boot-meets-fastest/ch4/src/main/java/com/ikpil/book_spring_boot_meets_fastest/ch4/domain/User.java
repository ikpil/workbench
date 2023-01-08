package com.ikpil.book_spring_boot_meets_fastest.ch4.domain;

import com.fasterxml.jackson.annotation.JsonIgnore;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.ToString;

import javax.persistence.*;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Entity
@Table(name = "users")
@ToString(exclude = "customers")
public class User {
    @Id // 기본키 설정
    private String username;

    @JsonIgnore // JPA와 관계 없지만, JSON 형식으로 출력할 때, 패스워드 필드를 감추기 위한 어노테이션
    private String encodedPassword;

    @JsonIgnore
    @OneToMany(cascade = CascadeType.ALL, fetch = FetchType.LAZY, mappedBy = "user") // 1:N 관계 정의
    private List<Customer> customers;
}
