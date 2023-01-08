package com.ikpil.hello.java.core;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Calendar;
import java.util.TimeZone;

public class TimeExample implements Example {
    private static final Logger logger = LoggerFactory.getLogger(CollectionExample.class);
    private static final TimeZone timeZone = TimeZone.getTimeZone("Asia/Seoul");

    public void run() {
        checkIntegerDate();
        checkIntegerYearWeek();
        checkTLSCalendar();
        checkIntegerYmd();
    }

    private void checkIntegerDate() {
        // 365 일 계산하여, 체크
        int prevDate = -1;
        for (int i = 1; i < 365 * 10; ++i) {
            Calendar calendar = Calendar.getInstance(timeZone);
            calendar.add(Calendar.DAY_OF_MONTH, i);

            int dateByCalendar = getIntegerDate(calendar);
            if (prevDate >= dateByCalendar) {
                logger.error("invalid date - prevDate({}) dateByCalendar({})", prevDate, dateByCalendar);
            }

            prevDate = dateByCalendar;
        }
    }

    private void checkIntegerYearWeek() {
        // 365 일 계산하여, 체크

        int prevDate = -1;
        for (int i = 1; i < 365 * 10; ++i) {
            Calendar calendar = Calendar.getInstance(timeZone);
            calendar.setFirstDayOfWeek(Calendar.MONDAY);
            calendar.setMinimalDaysInFirstWeek(4);
            calendar.add(Calendar.DAY_OF_MONTH, i);

            int dateByCalendar = getIntegerYearWeek(calendar);
            if (prevDate > dateByCalendar) {
                logger.error("invalid year week - date({}) dateByCalendar({})", calendar.getTime(), dateByCalendar);
            }

            prevDate = dateByCalendar;
        }
    }

    private void checkTLSCalendar() {
        Calendar cal1 = Calendar.getInstance(timeZone);
        Calendar cal2 = Calendar.getInstance(timeZone);

        int sec = cal1.get(Calendar.SECOND);
        try {
            Thread.sleep(2001L);
        } catch (Exception e) {
            // ...
        }
        cal1.setTimeInMillis(System.currentTimeMillis());
        int sec2 = cal1.get(Calendar.SECOND);

        int aaa = 3;
    }

    private void checkIntegerYmd() {
        if (201401 != getYearWeekByYmd(20140101)) {
            logger.error("invalid year week");
        }

        if (201501 != getYearWeekByYmd(20150101)) {
            logger.error("invalid year week");
        }

        // 2016년 01월 01은 금요일인데, 목요일이 없는 주차이므로, 전년도 53주차가 되어야함
        if (201553 != getYearWeekByYmd(20160101)) {
            logger.error("invalid year week");
        }

        // 2017년 01월 01은 일요일이고, 월요일이 1주차의 시작이므로, 2016년 52주차로 되어야 함
        if (201652 != getYearWeekByYmd(20170101)) {
            logger.error("invalid year week");
        }

        if (201801 != getYearWeekByYmd(20180101)) {
            logger.error("invalid year week");
        }

        if (201901 != getYearWeekByYmd(20190101)) {
            logger.error("invalid year week");
        }

        if (201936 != getYearWeekByYmd(20190902)) {
            logger.error("invalid year week");
        }
    }

    private int getIntegerDate(Calendar cal) {
        // Calendar.YEAR 의 필드는 1
        // Calendar.MONTH 의 필드는 0 ~ 11 까지의 값 *주의*
        // Calendar.DAY_OF_MONTH 의 필드는 1 ~ 31 까지의 값
        return cal.get(Calendar.YEAR) * 10000   // 20190000
                + (cal.get(Calendar.MONTH) + 1) * 100 // 20190700
                + cal.get(Calendar.DAY_OF_MONTH); // 20190723
    }

    private int getIntegerYearWeek(Calendar cal) {
        // 필독
        // 주차는 정확 하므로, 년도는 주차 기준으로 평가 해야 합니다.
        // 첫주의 시작은 월요일이며, 목요일이 있는 주가 1주차 계산 합니다.

        cal.setFirstDayOfWeek(Calendar.MONDAY);
        cal.setMinimalDaysInFirstWeek(Calendar.WEDNESDAY);

        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH) + 1; // *주의* 월은 0 ~ 11 까지이다.
        int weekOfYear = cal.get(Calendar.WEEK_OF_YEAR);

        if (1 == weekOfYear && 12 == month) {
            // 주차가 1인데, 12월일 경우, 다음년도 기준
            // 예) 2019년 12월 29일은 년도는 2019 이지만, 주차는 2020년 1주차 이다
            year += 1;
        } else if (52 <= weekOfYear && month == 1) {
            // 주차가 52 이상인데, 1월일 경우, 이전년도 셋팅
            // 예) 2021년 1월 1일은 년도는 2021년이지만, 주차는 2020년 53주차 이다.
            year -= 1;
        }

        return year * 100 + weekOfYear;
    }

    // Ymd 기준으로 Yw 반환
    public int getYearWeekByYmd(int Ymd) {
        final int year = Ymd / 10_000;
        final int month = ((Ymd / 100) % 100) - 1; // Note : month 는 -1 해야 캘린더와 동일합니다.
        final int day = Ymd % 100;

        // Ymd 를 기준으로, 주차를 계산한다.
        Calendar cal = Calendar.getInstance(timeZone);
        cal.set(year, month, day, 0, 0, 0);

        return getIntegerYearWeek(cal);
    }
}
