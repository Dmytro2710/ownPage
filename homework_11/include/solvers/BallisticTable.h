#pragma once
#include <vector>

#include <cmath>

struct BallisticTable {

    std::vector<float> axisZ0;
    std::vector<float> axisV0;
    std::vector<float> axisM;
    std::vector<float> axisD;
    std::vector<float> axisL;

    struct Result {
        float t;
        float hDist;
    };

    struct Interp {
        int lo;
        float frac;
    };

    std::vector<Result> data;

    size_t index(int iz, int iv, int im, int id, int il) const {
        return ((((size_t)iz * axisV0.size() + iv)
                              * axisM.size()  + im)
                              * axisD.size()  + id)
                              * axisL.size()  + il;
    }
 
    const Result& at(int iz, int iv, int im,
                     int id, int il) const {
        return data[index(iz, iv, im, id, il)];
    }

    bool load(const char* path);
    Result lookup(float Z0, float V0, float m, float d, float l) const;
private:
    static Result lerp(const Result& a, const Result& b, float t);
    static Interp findInterp(float val, const std::vector<float>& axis);    
};