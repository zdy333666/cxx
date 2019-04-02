//
// Created by zdy on 19-4-2.
//

/**
 *  https://blog.csdn.net/nk_wang/article/details/50405704
 */

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <string>
#include <iostream>
#include <sstream>

using namespace std;
using namespace boost::archive::iterators;

bool Base64Encode(const string &input, string *output) {

    typedef base64_from_binary<transform_width<string::const_iterator, 6, 8>> Base64EncodeIterator;
    stringstream result;

    try {
        copy(Base64EncodeIterator(input.begin()), Base64EncodeIterator(input.end()), ostream_iterator<char>(result));
    } catch (...) {
        return false;
    }

    size_t equal_count = (3 - input.length() % 3) % 3;
    for (size_t i = 0; i < equal_count; i++) {
        result.put('=');
    }

    *output = result.str();

    return output->empty() == false;
}

bool Base64Decode(const string &input, string *output) {

    typedef transform_width<binary_from_base64<string::const_iterator>, 8, 6> Base64DecodeIterator;
    stringstream result;

    try {
        copy(Base64DecodeIterator(input.begin()), Base64DecodeIterator(input.end()), ostream_iterator<char>(result));
    } catch (...) {
        return false;
    }

    *output = result.str();

    return output->empty() == false;
}


int main(int argc, char *argv[]) {

    cout << "C/C++中使用Base64编码解码(使用boost库)" << endl;

    string input_str("face://blog.csdn.net/qq0824?viewmode=contents");
    string base64_str, output_str;

    cout << "origin text:" << input_str << endl;

    Base64Encode(input_str, &base64_str);
    cout << "encode:" << base64_str << endl;

    Base64Decode(base64_str, &output_str);
    cout << "decode:" << output_str << endl;


    return 0;
}
