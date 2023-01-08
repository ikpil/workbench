package com.ikpil.book_spring_boot_meets_fastest.ch3;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.jdbc.DataSourceProperties;
import org.springframework.boot.jdbc.DataSourceBuilder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Primary;
import org.springframework.core.Ordered;
import org.springframework.core.annotation.Order;
import org.springframework.web.filter.CharacterEncodingFilter;

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
        //return new net.sf.log4jdbc.sql.jdbcapi.DataSourceSpy(this.dataSource);
        return this.dataSource;
    }

    /**
     * Order 애너테이션으로 highest precedence 로 순서를 설정했으며, 올림차 정렬로 순서를 매긴다.
     * 이 순서는 서블릿 필터의 적용 순서를 뜻한다, Bean 을 정의 함으로써 사용할 수 있다.
     * 한글이 깨지기 때문에 캐릭터 인코딩 필터링을 했다는데, 나는 한글이 깨지지 않았다.
     */
    @Order(Ordered.HIGHEST_PRECEDENCE)
    @Bean
    CharacterEncodingFilter characterEncodingFilter() {
        var filter = new CharacterEncodingFilter();
        filter.setEncoding("UTF-8");
        filter.setForceEncoding(true);

        return filter;
    }
}
