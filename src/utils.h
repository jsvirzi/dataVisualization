//
// Created by jsvirzi on 4/2/17.
//

#ifndef UTILS_H
#define UTILS_H

bool readFieldFromCsv(const char *line, int field, char *res, int max);
time_t hourTimeBaseInSeconds(const char *dateStr, const char *timeStr);

#endif //UTILS_H
