//
// Created by jsvirzi on 4/2/17.
//

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


