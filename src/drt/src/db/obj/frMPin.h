/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FR_MPIN_H_
#define _FR_MPIN_H_

#include <iostream>

#include "db/obj/frPin.h"
#include "db/obj/frAccess.h"
#include "db/obj/frShape.h"
#include "frBaseTypes.h"

namespace fr {
class frMTerm;

class frMPin : public frPin
{
 public:
  // constructors
  frMPin() : frPin(), term_(nullptr), aps_() {}
  frMPin(const frMPin& in) : frPin(in), term_(in.term_), aps_() {}
  frMPin(const frMPin& in, const dbTransform& xform)
      : frPin(in, xform), term_(in.term_), aps_() {}

  // getters
  frMTerm* getTerm() const { return term_; }

  int getNumPinAccess() const override { return aps_.size(); }
  bool hasPinAccess() const override { return !aps_.empty(); }
  frPinAccess* getPinAccess(int idx) const { return aps_[idx].get(); }

  // setters
  // cannot have setterm, must be available when creating
  void setTerm(frMTerm* in) { term_ = in; }
  void addPinAccess(std::unique_ptr<frPinAccess> in)
  {
    in->setId(aps_.size());
    aps_.push_back(std::move(in));
  }
  // others
  frBlockObjectEnum typeId() const override { return frcPin; }

 protected:
  frMTerm* term_;
  std::vector<std::unique_ptr<frPinAccess>>
      aps_;  // not copied in copy constructor
};
}  // namespace fr

#endif
