#include "../include/coppeliasim_plugin_velodyne/velodyneROSModelCont.h"
#include "../include/v_repLib.h"

CVelodyneROSModelCont::CVelodyneROSModelCont()
{
}

CVelodyneROSModelCont::~CVelodyneROSModelCont()
{
    removeAll();
}

int CVelodyneROSModelCont::addObject(CVelodyneROSModel *obj)
{
    _allObjects.push_back(obj);
    return (obj->getVelodyneHandle());
}

bool CVelodyneROSModelCont::removeObject(int velodyneHandle)
{
    for (int i = 0; i < int(_allObjects.size()); i++)
    {
        if (_allObjects[i]->getVelodyneHandle() == velodyneHandle)
        {
            delete _allObjects[i];
            _allObjects.erase(_allObjects.begin() + i);
            return (true);
        }
    }
    return (false);
}

void CVelodyneROSModelCont::removeAll()
{
    for (int i = 0; i < int(_allObjects.size()); i++)
        delete _allObjects[i];
    _allObjects.clear();
}

CVelodyneROSModel *CVelodyneROSModelCont::getObject(int velodyneHandle)
{
    for (int i = 0; i < int(_allObjects.size()); i++)
    {
        if (_allObjects[i]->getVelodyneHandle() == velodyneHandle)
            return (_allObjects[i]);
    }
    return (0);
}
