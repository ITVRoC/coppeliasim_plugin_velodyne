#pragma once

#include "velodyneROSModel.h"
#include <vector>

class CVelodyneROSModelCont
{
  public:
    CVelodyneROSModelCont();

    virtual ~CVelodyneROSModelCont();

    int addObject(CVelodyneROSModel *obj);

    CVelodyneROSModel *getObject(int velodyneHandle);

    bool removeObject(int velodyneHandle);

    void removeAll();

  private:
    std::vector<CVelodyneROSModel *> _allObjects;
};
