#ifndef MODEL_ABSTRACT
#define MODEL_ABSTRACT

class ModelAbstract
{
private:
    /* data */
public:
    ModelAbstract(/* args */) {}

    virtual void solve() = 0;
    virtual void find_violated_constraints() = 0;
    ~ModelAbstract() {}
};

#endif