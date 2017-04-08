#include <time.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "utils.h"

/* this is brute force calculated / entered by hand. it works for 2017 */

const int weeksUntil2017 = 1930; /* Jan 1, 2017 is the beginning of gps reference week 1930 */

static struct simpleDate {
    int gpsWeek, year, month, day;
} referenceWeeksToSimpleDate[] = {
    { 1930, 2017, 1, 1},
    { 1931, 2017, 1, 8},
    { 1932, 2017, 1, 15},
    { 1933, 2017, 1, 22},
    { 1934, 2017, 1, 29},
    { 1935, 2017, 2, 5},
    { 1936, 2017, 2, 12},
    { 1937, 2017, 2, 19},
    { 1938, 2017, 2, 26},
    { 1939, 2017, 3, 5},
    { 1940, 2017, 3, 12},
    { 1941, 2017, 3, 19},
    { 1942, 2017, 3, 26},
    { 1943, 2017, 4, 2},
    { 1944, 2017, 4, 9},
    { 1945, 2017, 4, 16},
    { 1946, 2017, 4, 23},
    { 1947, 2017, 4, 30},
    { 1948, 2017, 5, 7},
    { 1949, 2017, 5, 14},
    { 1950, 2017, 5, 21},
    { 1951, 2017, 5, 28},
    { 1952, 2017, 6, 4},
    { 1953, 2017, 6, 11},
    { 1954, 2017, 6, 18},
    { 1955, 2017, 6, 25},
    { 1956, 2017, 7, 2},
    { 1957, 2017, 7, 9},
    { 1958, 2017, 7, 16},
    { 1959, 2017, 7, 23},
    { 1960, 2017, 7, 30},
    { 1961, 2017, 8, 6},
    { 1962, 2017, 8, 13},
    { 1963, 2017, 8, 20},
    { 1964, 2017, 8, 27},
    { 1965, 2017, 9, 3},
    { 1966, 2017, 9, 10},
    { 1967, 2017, 9, 17},
    { 1968, 2017, 9, 24},
    { 1969, 2017, 10, 1},
    { 1970, 2017, 10, 8},
    { 1971, 2017, 10, 15},
    { 1972, 2017, 10, 22},
    { 1973, 2017, 10, 29},
    { 1974, 2017, 11, 5},
    { 1975, 2017, 11, 12},
    { 1976, 2017, 11, 19},
    { 1977, 2017, 11, 26},
    { 1978, 2017, 12, 3},
    { 1979, 2017, 12, 10},
    { 1980, 2017, 12, 17},
    { 1981, 2017, 12, 24},
    { 1982, 2017, 12, 31},
    { 0, 0, 0, 0}
};

uint64_t secondsAtStartOfReferenceWeek(int week) {
    int index = week - 1930;
    if(referenceWeeksToSimpleDate[index].gpsWeek != week) return 0; /* ensure consistency */
    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_mday = referenceWeeksToSimpleDate[index].day;
    t.tm_mon = referenceWeeksToSimpleDate[index].month;
    t.tm_year = referenceWeeksToSimpleDate[index].year;
    time_t seconds = mktime(&t);
    return seconds;
}
/* taken from https://www.ngs.noaa.gov/CORS/Gpscal.shtml */

/*
 * GPS Calendar

This calendar will help you convert a calendar date to either the Day of Year or GPS Week #.
For example, February 4, 2016 is day of year 35 in GPS Week 1882.
The GPS Week # would be 18824 (the # 4 represents Thursday).

Sunday=0, Monday=1, Tuesday=2, Wednesday=3, Thursday=4, Friday=5, Saturday=6

2017 | 2016 | 2015 | 2014 | 2013 | 2012 | 2011 | 2010 | 2009 | 2008 | 2007 | 2006

Interactive Calendar - leaves NGS

Dec             2016                 Dec
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1925                    1   2   3    1925                  336 337 338
1926    4   5   6   7   8   9  10    1926  339 340 341 342 343 344 345
1927   11  12  13  14  15  16  17    1927  346 347 348 349 350 351 352
1928   18  19  20  21  22  23  24    1928  353 354 355 356 357 358 359
1929   25  26  27  28  29  30  31    1929  360 361 362 363 364 365 366


Jan             2017                 Jan
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1930    1   2   3   4   5   6   7    1930    1   2   3   4   5   6   7
1931    8   9  10  11  12  13  14    1931    8   9  10  11  12  13  14
1932   15  16  17  18  19  20  21    1932   15  16  17  18  19  20  21
1933   22  23  24  25  26  27  28    1933   22  23  24  25  26  27  28
1934   29  30  31                    1934   29  30  31


Feb             2017                 Feb
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1934                1   2   3   4    1934               32  33  34  35
1935    5   6   7   8   9  10  11    1935   36  37  38  39  40  41  42
1936   12  13  14  15  16  17  18    1936   43  44  45  46  47  48  49
1937   19  20  21  22  23  24  25    1937   50  51  52  53  54  55  56
1938   26  27  28                    1938   57  58  59


Mar             2017                 Mar
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1938                1   2   3   4    1938               60  61  62  63
1939    5   6   7   8   9  10  11    1939   64  65  66  67  68  69  70
1940   12  13  14  15  16  17  18    1940   71  72  73  74  75  76  77
1941   19  20  21  22  23  24  25    1941   78  79  80  81  82  83  84
1942   26  27  28  29  30  31        1942   85  86  87  88  89  90


Apr             2017                 Apr
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1942                            1    1942                           91
1943    2   3   4   5   6   7   8    1943   92  93  94  95  96  97  98
1944    9  10  11  12  13  14  15    1944   99 100 101 102 103 104 105
1945   16  17  18  19  20  21  22    1945  106 107 108 109 110 111 112
1946   23  24  25  26  27  28  29    1946  113 114 115 116 117 118 119
1947   30                            1947  120

May             2017                 May
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1947        1   2   3   4   5   6    1947      121 122 123 124 125 126
1948    7   8   9  10  11  12  13    1948  127 128 129 130 131 132 133
1949   14  15  16  17  18  19  20    1949  134 135 136 137 138 139 140
1950   21  22  23  24  25  26  27    1950  141 142 143 144 145 146 147
1951   28  29  30  31                1951  148 149 150 151


Jun             2017                 Jun
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1951                    1   2   3    1951                  152 153 154
1952    4   5   6   7   8   9  10    1952  155 156 157 158 159 160 161
1953   11  12  13  14  15  16  17    1953  162 163 164 165 166 167 168
1954   18  19  20  21  22  23  24    1954  169 170 171 172 173 174 175
1955   25  26  27  28  29  30        1955  176 177 178 179 180 181


Jul             2017                 Jul
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1955                            1    1955                          182
1956    2   3   4   5   6   7   8    1956  183 184 185 186 187 188 189
1957    9  10  11  12  13  14  15    1957  190 191 192 193 194 195 196
1958   16  17  18  19  20  21  22    1958  197 198 199 200 201 202 203
1959   23  24  25  26  27  28  29    1959  204 205 206 207 208 209 210
1960   30  31                        1960  211 212

Aug             2017                 Aug
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1960            1   2   3   4   5    1960          213 214 215 216 217
1961    6   7   8   9  10  11  12    1961  218 219 220 221 222 223 224
1962   13  14  15  16  17  18  19    1962  225 226 227 228 229 230 231
1963   20  21  22  23  24  25  26    1963  232 233 234 235 236 237 238
1964   27  28  29  30  31            1964  239 240 241 242 243


Sep             2017                 Sep
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1964                        1   2    1964                      244 245
1965    3   4   5   6   7   8   9    1965  246 247 248 249 250 251 252
1966   10  11  12  13  14  15  16    1966  253 254 255 256 257 258 259
1967   17  18  19  20  21  22  23    1967  260 261 262 263 264 265 266
1968   24  25  26  27  28  29  30    1968  267 268 269 270 271 272 273


Oct             2017                 Oct
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1969    1   2   3   4   5   6   7    1969  274 275 276 277 278 279 280
1970    8   9  10  11  12  13  14    1970  281 282 283 284 285 286 287
1971   15  16  17  18  19  20  21    1971  288 289 290 291 292 293 294
1972   22  23  24  25  26  27  28    1972  295 296 297 298 299 300 301
1973   29  30  31                    1973  302 303 304


Nov             2017                 Nov
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1973                1   2   3   4    1973              305 306 307 308
1974    5   6   7   8   9  10  11    1974  309 310 311 312 313 314 315
1975   12  13  14  15  16  17  18    1975  316 317 318 319 320 321 322
1976   19  20  21  22  23  24  25    1976  323 324 325 326 327 328 329
1977   26  27  28  29  30            1977  330 331 332 333 334


Dec             2017                 Dec
    GPS WK Sun Mon Tue Wed Thu Fri Sat   GPS WK Sun Mon Tue Wed Thu Fri Sat
1977                        1   2    1977                      335 336
1978    3   4   5   6   7   8   9    1978  337 338 339 340 341 342 343
1979   10  11  12  13  14  15  16    1979  344 345 346 347 348 349 350
1980   17  18  19  20  21  22  23    1980  351 352 353 354 355 356 357
1981   24  25  26  27  28  29  30    1981  358 359 360 361 362 363 364
1982   31                            1982  365

 *
 */

int GpsTimeUtc(const char *filename) {
    int i, skip = 50;
    size_t maxLineDim = 1024;
    FILE *fpi = fopen(filename, "r");
    bool debug = false, verbose = false;
    if(fpi == 0) {
        printf("unable to open file [%s]\n", filename);
        return -1;
    }

	int utcHour, utcMins, utcSecs, utcMsecs, gpsWeek, gpsTime;
	struct tm utcTime;
	bool trigger = false;
    char msgId[32];
    char *line = (char *)malloc(maxLineDim);
    for(i=0;i<skip;++i) { getline(&line, &maxLineDim, fpi); }

    while(getline(&line, &maxLineDim, fpi) != -1) {
        int length = strlen(line);
        char ch = line[length-1];
        if((ch == '\n') || (ch == '\r')) { line[length-1] = 0; }
        readFieldFromCsv(line, 0, msgId, sizeof(msgId));
        if(strcmp(msgId, "#INSPVAXA") == 0) {
			if(trigger) {
				gpsWeek = readIntFromCsv(line, 5);
				gpsTime = readDoubleFromCsv(line, 6);
				gpsTime = floor(gpsTime);
				break;
			}
        } else if(strcmp(msgId, "$GPRMC") == 0) {
			char timeStr[32], dateStr[32], str[3];
			readFieldFromCsv(line, 1, timeStr, sizeof(timeStr));
			readFieldFromCsv(line, 9, dateStr, sizeof(dateStr));
			str[2] = 0;
			str[0] = timeStr[0];
			str[1] = timeStr[1];
			sscanf(str, "%2d", &utcHour);
			str[0] = timeStr[2];
			str[1] = timeStr[3];
			sscanf(str, "%2d", &utcMins);
			str[0] = timeStr[4];
			str[1] = timeStr[5];
			sscanf(str, "%2d", &utcSecs);
			str[0] = timeStr[7];
			str[1] = timeStr[8];
			sscanf(str, "%2d", &utcMsecs);
			if(utcMsecs == 50) {
				memset(&utcTime, 0, sizeof(struct tm));
				str[0] = dateStr[0];
				str[1] = dateStr[1];
				sscanf(str, "%02d", &utcTime.tm_mday);
				str[0] = dateStr[2];
				str[1] = dateStr[3];
				sscanf(str, "%02d", &utcTime.tm_mon);
				str[0] = dateStr[4];
				str[1] = dateStr[5];
				sscanf(str, "%02d", &utcTime.tm_year);
				utcTime.tm_year += 2000;
				utcTime.tm_hour = utcHour;
				utcTime.tm_min = utcMins;
				utcTime.tm_sec = utcSecs;
				trigger = true;
			}
        }
    }

    free(line);

	int gpsSeconds = secondsAtStartOfReferenceWeek(gpsWeek) + gpsTime;
	int utcSeconds = mktime(&utcTime);
	int diff = gpsSeconds - utcSeconds;
	return diff;
}