package com.ikpil.hello.java.core;

import org.redisson.Redisson;
import org.redisson.api.*;
import org.redisson.client.RedisConnectionException;
import org.redisson.client.protocol.ScoredEntry;
import org.redisson.config.Config;
import org.redisson.config.TransportMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

/**
 * redis-server
 * windows : https://github.com/microsoftarchive/redis/releases
 * <p>
 * redis-client
 * redisson : https://github.com/redisson/redisson
 */
public class RedissonExample implements Example {
    private static final Logger logger = LoggerFactory.getLogger(RedissonExample.class);
    //private static final String mainHost = "redis://127.0.0.1:6379";
    private static final String mainHost = "redis://192.168.0.240:6379";

    public void run() {
        try {
            consume(this::connectAndShutdown);
            consume(this::getOrSet);
            consume(this::publishOrSubscribe);
            consume(this::map);
            consume(this::sortedSet);
            consume(this::scoredSortedSet);
        } catch (Exception e) {
            // 레디스를 로컬에 구성하고 테스트 할 것
            logger.error("", e);
        }
    }

    private RedissonClient create(String host) {
        // Programmatic configuration
        Config config = new Config();
        config.setNettyThreads(64);

        if (OSValidator.isUnix()) {
            config.setTransportMode(TransportMode.EPOLL);
        } else {
            config.setTransportMode(TransportMode.NIO);
        }

        // use "rediss://" for SSL connection
        config.useSingleServer()
                .setAddress(host);

        return Redisson.create(config);
    }

    private void consume(Consumer<RedissonClient> consumer) throws RedisConnectionException {
        RedissonClient redisson = null;
        try {
            redisson = create(mainHost);
            if (null == redisson) {
                return;
            }

            consumer.accept(redisson);

        } finally {
            if (null == redisson) {
                logger.error("redisson creation failed");
            } else {
                redisson.shutdown();
            }
        }
    }

    // 케넥션 테스트, 할 일이 없다
    private void connectAndShutdown(RedissonClient redisson) {
    }

    // get set 예제
    private void getOrSet(RedissonClient redisson) {
        String key = "getOrSet";
        long setValue = 1234L;
        redisson.getBucket(key).set(setValue, 10, TimeUnit.SECONDS);
        logger.info("redis - set getOrSet {}", setValue);

        long getValue = (Long) redisson.getBucket(key).get();
        logger.info("redis - get getOrSet {}", getValue);
    }


    /**
     * 테스트 케이스
     * - localhost, Windows10 1809 64bit, i5-2500, DDR3 24G
     * -  1 thread, 4초간 13141회 수신, 초당 약 3200회 수신 가능
     * -  8 thread, 4초간 17021회 수신, 초당 약 4200회 수신 가능
     * - 10 thread, 4초간 15209회 수신, 초당 약 3800회 수신 가능
     * -
     * 테스트 케이스 2
     * -
     * -
     * 테스트 결과
     * -
     * publish -> publishAsync 가 10% ~ 20% 더 빠른 결과
     * 하나의 클라이언트가 수신 토픽 갯수를 늘린다고, 수신 반응이 좋아지지 않음
     * 하나의 클라이언트가 송신 토픽 갯수를 늘린다고, 송신 반응이 좋아지지 않음
     * -
     * 복수의 클라이언트로 쓰레드의 갯수를 늘리면 반응은 좋아지지만,
     * 일반적으로 사용할 수 없는 구조(송신 패킷 순서가 의도와 달라질 수 있음)
     * -
     * -
     * 결론
     * -
     * 효율 측면에서
     * 하나의 클라이언트로 처리하는게 최고 효율
     * -
     * 성능 측면에서
     * 복수의 클라이언트가 서로 다른 토픽으로 분리해서 받는다면, 최고 성능
     */
    // publish/subscribe 예제
    private void publishOrSubscribe(RedissonClient client) {
        String topicName = "publishOrSubscribe";

        // 카운트 다운 등록
        final int countdown = 100000;
        AtomicLong listenCnt = new AtomicLong();
        AtomicBoolean isExeuting = new AtomicBoolean(true);

        // 리스너 등록
        int threadCnt = 1;
        for (int i = 0; i < threadCnt; ++i) {
            client.getTopic(topicName + i).addListener(
                    String.class,
                    (channel, msg) -> listenCnt.incrementAndGet()
            );
        }
        logger.info("subscribe - key({}) count({})", topicName, countdown);


        // 쓰레드 돌림
        ArrayList<Thread> threads = new ArrayList<>();
        for (int threadIdx = 0; threadIdx < threadCnt; ++threadIdx) {
            Thread t = new Thread(() -> publisher(
                    isExeuting, topicName, threadCnt, countdown
            ));
            threads.add(t);
        }

        try {
            for (Thread t : threads) {
                t.start();
            }

            Thread.sleep(1000L);
            isExeuting.compareAndSet(true, false);

        } catch (InterruptedException e) {
            logger.error("", e);
        }

        logger.info("published - key({}) send({}) recv({})", topicName, countdown, listenCnt.get());
    }

    // topic publisher
    private void publisher(AtomicBoolean isExeuting, String topicName, int threadCnt, int countdown) {
        // 퍼블리쉬 시작
        RedissonClient publishClient = create(mainHost);

        ArrayList<RTopic> topics = new ArrayList<>();
        for (int i = 0; i < threadCnt; ++i) {
            RTopic topic = publishClient.getTopic(topicName + i);
            topics.add(topic);
        }

        for (int i = 0; isExeuting.get() && i < countdown / threadCnt + 1; ++i) {
            topics.get(i % topics.size()).publishAsync("z");
            //publishTopic.publish("z");

            try {
                // 10번당 한번씩 쉰다
                if (0 == (i % 10)) {
                    Thread.sleep(0L, 1);
                }
            } catch (Exception e) {
                logger.error("", e);
            }
        }

        publishClient.shutdown();
    }

    // 맵 사용 예제 추가
    private void map(RedissonClient redisson) {
        String name = "map";
        redisson.getBucket(name).delete();

        RMap<String, String> map = redisson.getMap(name);
        map.expire(10, TimeUnit.SECONDS);

        // 주요기능 : 넣을 때, 삭제 할 때 이전 값이 반환 됨을 알 수 있다.
        {
            String first = map.put("123", "prev");
            String second = map.put("123", "current");
            String third = map.remove("123");
            logger.info("put & remove check - first({}) second({}) third({})", first, second, third);
        }

        // 주요 기능 : 같은 키에 값이 있을 때, 들어가지 않음을 알 수 있다.
        {
            String first = map.putIfAbsent("323", "prev");
            String second = map.putIfAbsent("323", "current");
            String third = map.remove("323");
            logger.info("putIfAbsent & remove check - first({}) second({}) third({})", first, second, third);
        }

        // use fast* methods when previous value is not required
        // 주요 기능 : 이전 값이 필요 없다면, fastPut 을 이용할 것

        // 같은 값이 있을 경우, 넣어지지만 반환은 false 이다.
        {
            boolean first = map.fastPut("a", "fast put 1 of a");
            boolean second = map.fastPut("a", "fast put 2 of a");
            String v = map.get("a");
            logger.info("fast put check - first({}) second({}) final value({})", first, second, v);
        }

        // 주요 기능 : 같은 값은 안 넣을 수 있고, 그 결과값을 받아서 볼 수 있다.
        {
            boolean first = map.fastPutIfAbsent("d", "fast put if absent 1 of d");
            boolean second = map.fastPutIfAbsent("d", "fast put if absent 2 of d");
            String v = map.get("d");
            logger.info("fast put if absent check - first({}) second({}) value({})", first, second, v);
        }

        map.fastRemove("a", "d");

        // 비동기 작업 테스트
        {
            RFuture<String> firstFuture = map.putAsync("321", "put async 1 of 321");
            RFuture<Boolean> secondFuture = map.fastPutAsync("321", "fast put async 2 of 321");
            RFuture<Boolean> thirdFuture = map.fastPutAsync("321", "fast put async 3 of 321");
            RFuture<Long> fourFuture = map.fastRemoveAsync("321");
            logger.info("asyc check - {}", map.readAllMap());

            try {
                fourFuture.await();
            } catch (InterruptedException e) {
                logger.error("", e);
            }
        }


    }

    // https://github.com/redisson/redisson/wiki/7.-distributed-collections#74-sortedset
    private void sortedSet(RedissonClient redisson) {
        String name = "sortedSet";
        RSortedSet<Integer> set = redisson.getSortedSet(name);
        redisson.getBucket(name).expire(10, TimeUnit.SECONDS);

        set.add(0);
        set.add(3);
        set.add(1);
        set.add(2);
        set.add(3);

        set.add(5);

        /*
         * 입력값에 의한 정렬
         * 중복값 안 들어감
         */
        logger.info("set entries - {}", set.readAll());
    }


    // https://github.com/redisson/redisson/wiki/7.-distributed-collections#75-scoredsortedset
    private void scoredSortedSet(RedissonClient redisson) {
        String name = "scoredSortedSet";
        redisson.getBucket(name).delete();

        RScoredSortedSet<String> set = redisson.getScoredSortedSet(name);
        set.expire(10, TimeUnit.SECONDS);

        long score = System.nanoTime();
        for (int i = 0; i < 50; ++i) {
            set.add(score - i, "A" + i);
        }
        int setSize = set.size();

        // 총 엔트리
        // 스코어가 작을 상위 랭크이므로, 올림차 정렬임을 알 수 있습니다.
        logger.info("scored sorted set entries - size({}) entries({})", setSize, set.readAll());

        int index = set.rank("A10");
        logger.info("A10 rank - index({})", index);

        long ascore = set.getScore("A10").longValue();
        logger.info("A10 score - begin score({}) score({})", score, ascore);

        // 주요 기능 : 페이지 단위로 얻어 올 수있음, 0 ~ 20 개이므로 총 21개
        Collection<ScoredEntry<String>> entryGroup1 = set.entryRange(0, 20);
        ArrayList<String> values = new ArrayList<>();
        entryGroup1.forEach(entry -> values.add(entry.getValue()));
        logger.info("scored sorted set group 1 entries - size({}) entries({})", entryGroup1.size(), values);

        // 주요 기능 : 인덱스를 뒤집어(reversed) 가져 올 수 있음
        Collection<ScoredEntry<String>> reversedEntryGroup = set.entryRangeReversed(0, 20);
        ArrayList<String> reversedValues = new ArrayList<>();
        reversedEntryGroup.forEach(entry -> reversedValues.add(entry.getValue()));
        logger.info("scored sorted set group reversed entries - size({}) entries({})", reversedEntryGroup.size(), reversedValues);

        // 인덱스를 벗어 났을 경우, 값을 가져 오지 않는다.
        Collection<ScoredEntry<String>> emptyEntryGroup = set.entryRange(setSize, setSize + 20);
        ArrayList<String> emptyValues = new ArrayList<>();
        emptyEntryGroup.forEach(entry -> emptyValues.add(entry.getValue()));
        logger.info("scored sorted set empty group entries - size({}) entries({})", emptyEntryGroup.size(), emptyValues);
    }
}
