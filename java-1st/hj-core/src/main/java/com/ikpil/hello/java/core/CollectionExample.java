package com.ikpil.hello.java.core;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.CopyOnWriteArraySet;

public class CollectionExample implements Example {
    private static final Logger logger = LoggerFactory.getLogger(CollectionExample.class);

    public void run() {
        String key1 = "가";
        String key2 = "나";
        String key3 = "다";

        HashMap<String, Integer> hm = new HashMap<>();
        hm.put(key1, 1);
        hm.put(key2, 2);
        hm.put(key3, 3);
        logger.info("hash map - {}", hm);

        TreeMap<String, Integer> tm = new TreeMap<>();
        tm.put(key1, 1);
        tm.put(key2, 2);
        tm.put(key3, 3);
        logger.info("tree map - {}", tm);

        LinkedHashMap<String, Integer> lhm = new LinkedHashMap<>();
        lhm.put(key1, 1);
        lhm.put(key2, 2);
        lhm.put(key3, 3);
        logger.info("linked hash map - {}", lhm);

        ConcurrentHashMap<String, Integer> chm = new ConcurrentHashMap<>();
        chm.put(key1, 1);
        chm.put(key2, 2);
        chm.put(key3, 3);
        logger.info("thread safe hash map - {}", chm);

        ConcurrentSkipListMap<String, Integer> cslm = new ConcurrentSkipListMap<>();
        cslm.put(key1, 1);
        cslm.put(key2, 2);
        cslm.put(key3, 3);
        logger.info("thread safe skip list map - {}", cslm);

        CopyOnWriteArraySet<String> cowas = new CopyOnWriteArraySet<>();
        cowas.add(key1);
        cowas.add(key2);
        cowas.add(key3);
        logger.info("thread safe copy on write array set - {}", cowas);
    }
}
