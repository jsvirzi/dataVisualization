//
// Created by jsvirzi on 4/2/17.
//

#include <string.h>
#include <time.h>
#include <stdio.h>

bool readFieldFromCsv(const char *line, int field, char *res, int max) {
    int i, pos = 0, len = strlen(line), nCommas = 0, beginPos = 0, endPos = -1;
    for(pos=0;pos<=len;++pos) {
        char ch = line[pos];
        if((ch == ',') || (ch == 0)) {
            if(nCommas == field) {
                endPos = pos;
            } else {
                beginPos = pos + 1;
            }

            if(endPos >= 0) {
                int tLen = endPos - beginPos;
                if(tLen < max) {
                    for(i=0;i<tLen;++i) {
                        res[i] = line[beginPos + i];
                    }
                    res[i] = 0;
                    return true;
                } else {
                    return false;
                }
            }
            ++nCommas;
        }
    }
    return false;
}

/*
 * given a date string in form 190264 and time 061234.00
 * 19 Feb 1964 6:12:34am
 * returns the number of seconds since the epoch Jan 1, 1970
 */
time_t hourTimeBaseInSeconds(const char *dateStr, const char *timeStr) {
    struct tm t;
    memset(&t, 0, sizeof(struct tm));
    char str[3];
    str[2] = 0;
    str[0] = dateStr[0];
    str[1] = dateStr[1];
    sscanf(str, "%02d", &t.tm_mday);
    str[0] = dateStr[2];
    str[1] = dateStr[3];
    sscanf(str, "%02d", &t.tm_mon);
    str[0] = dateStr[4];
    str[1] = dateStr[5];
    sscanf(str, "%02d", &t.tm_year);
    t.tm_year += 2000; /* 2017 comes as 17 */
    str[0] = timeStr[0];
    str[1] = timeStr[1];
    sscanf(str, "%02d", &t.tm_hour);
    time_t seconds = mktime(&t);
    return seconds;
}

