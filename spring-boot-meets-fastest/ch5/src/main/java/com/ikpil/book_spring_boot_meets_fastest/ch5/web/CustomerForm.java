package com.ikpil.book_spring_boot_meets_fastest.ch5.web;

import lombok.Data;

import javax.validation.constraints.NotNull;
import javax.validation.constraints.Size;

@Data // getter/setter 셋팅 하지 않아도 됨
public class CustomerForm {
    @NotNull
    @Size(min = 1, max = 127)
    private String firstName;

    @NotNull
    @Size(min = 1, max = 127)
    private String lastName;
}
