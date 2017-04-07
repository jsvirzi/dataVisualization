//
// Created by jsvirzi on 4/2/17.
//

#ifndef UTILS_H
#define UTILS_H

bool readFieldFromCsv(const char *line, int field, char *res, int max);
time_t hourTimeBaseInSeconds(const char *dateStr, const char *timeStr);
double readDoubleFromCsv(const char *line, int field);
int readIntFromCsv(const char *line, int field);
int readUint64FromCsv(const char *line, int field);

const double InvalidDouble = -99999999.9;
const int InvalidInt = -99999999;

#endif //UTILS_H
