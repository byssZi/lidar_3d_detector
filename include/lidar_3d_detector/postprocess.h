#ifndef POSTPROCESS_H_
#define POSTPROCESS_H_

#include <vector>

struct Bndbox {
    float x;
    float y;
    float z;
    float w;
    float l;
    float h;
    float rt;
    int id;
    float score;
    Bndbox(){};
    Bndbox(float x_, float y_, float z_, float l_, float w_, float h_, float rt_, int id_, float score_)
        : x(x_), y(y_), z(z_), w(w_), l(l_), h(h_), rt(rt_), id(id_), score(score_) {}
};

int nms_cpu(std::vector<Bndbox> bndboxes, const float nms_thresh,
            std::vector<Bndbox> &nms_pred, const int pre_nms_top_n);

#endif
