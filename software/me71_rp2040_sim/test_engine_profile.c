#include "engine_profile.h"
#include <assert.h>
int main(void){
  engine_shape_t s={{120,60,false},{480,60,false},{60,2,false}};
  for (unsigned i=0;i<120;i++) {
    unsigned word=engine_slot_word(&s,i);
    assert((word&8u)==(i==0?8u:0u));
    assert(((word>>4u)&8u)==(i==0?8u:0u));
    assert((word&1u)==((i%60u)<58u?1u:0u));
    assert(!((word>>4u)&1u));
  }
  crank_shape_t wheels[]={{36,1,false},{60,2,false},{60,2,true},{24,2,false},{120,16,false}};
  for(unsigned j=0;j<sizeof(wheels)/sizeof(wheels[0]);j++) {
    s.crank=wheels[j];
    assert(crank_shape_valid(&s.crank));
    for(unsigned slot=0;slot<engine_cycle_slots(&s);slot++) {
      unsigned word=engine_slot_word(&s,slot);
      unsigned tooth=(slot%s.crank.teeth)<(unsigned)(s.crank.teeth-s.crank.missing);
      assert((word&1u)==(tooth != s.crank.invert));
      assert(((word>>4)&1u)==s.crank.invert);
      assert(!!(word&8u)==(slot==0u));
    }
  }
  return 0;
}
