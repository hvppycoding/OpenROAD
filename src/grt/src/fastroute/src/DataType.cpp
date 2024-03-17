#include "DataType.h"
#include "odb/db.h"

namespace grt {

uint FrPin::pinId() const { 
    if (is_port_) 
        return bterm_->getId(); 
    else 
        return iterm_->getId();
}

odb::dbITerm* FrPin::getITerm() const {
    if (is_port_) 
        return nullptr; 
    else 
        return iterm_; 
}

odb::dbBTerm* FrPin::getBTerm() const {
    if (is_port_) 
        return bterm_; 
    else 
        return nullptr; 
}

int FrPin::instId() const { 
    if (is_port_) 
        return bterm_->getId();
    else 
        return iterm_->getInst()->getId();
}

bool FrPin::isSequential() const { 
    if (is_port_) 
        return true;
    else 
        return iterm_->getInst()->getMaster()->isSequential();
}

std::string FrPin::pinName() const { 
    if (is_port_) 
        return bterm_->getName();
    else 
        return iterm_->getMTerm()->getName();
}

std::string FrPin::instName() const { 
    if (is_port_) 
        return bterm_->getName();
    else
        return iterm_->getInst()->getName();
}

}