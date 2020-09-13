#include "ParameterStore.h"

ParameterStore::ParameterStore(uint16_t size){
    this->values = new float[size];
    this->size = size;
    for (int j=0;j<size;j++){
        this->values[j] = 0.0;
    }
}

ParameterStore::~ParameterStore(){
    delete values;
}

void ParameterStore::setValue(uint16_t id, float value){
    if (id < this->size ) {
        values[id] = value;
    }
}

float ParameterStore::getValue(uint16_t id) {
    return (id < this->size) ? values[id] : 0.0;
}