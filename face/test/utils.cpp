//
// Created by zdy on 19-4-6.
//

namespace utils {

    long timestamp() {

        struct timeval tv;
        gettimeofday(&tv, NULL);

        return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
    }

}