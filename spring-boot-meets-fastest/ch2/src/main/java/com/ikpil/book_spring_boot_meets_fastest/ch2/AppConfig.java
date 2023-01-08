package com.ikpil.book_spring_boot_meets_fastest.ch2;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.jdbc.DataSourceProperties;
import org.springframework.boot.jdbc.DataSourceBuilder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Primary;

import javax.sql.DataSource;

@Configuration
public class AppConfig {
    @Autowired
    DataSourceProperties dataSourceProperties;
    DataSource dataSource;

    @Bean
    @Primary
    DataSource dataSource() {
        var factory = DataSourceBuilder
                .create(this.dataSourceProperties.getClassLoader())
                .url(this.dataSourceProperties.getUrl())
                .username(this.dataSourceProperties.getUsername())
                .password(this.dataSourceProperties.getPassword());

        this.dataSource = factory.build();
        return new net.sf.log4jdbc.sql.jdbcapi.DataSourceSpy(this.dataSource);
    }
}
