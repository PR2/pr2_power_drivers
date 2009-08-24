
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include "ocean.h"
#include <time.h>

using namespace std;
using namespace willowgarage::ocean;

int main(int argc, char** argv)
{
  //printf("device=%s  debug_level=%d\n", argv[1], atoi(argv[2]));
  cout << "device=" << argv[1] <<  "  debug_level=" << atoi(argv[2]) << endl;

  ocean os( atoi(argv[2]));

  os.initialize(argv[1]);

  //int count = 10;
  while(1)
  {
    os.run();
    usleep(100);
#if 0
    --count;
    if(count == 0)
    {
      count = 10;
      
      cout << "------------------------------\n";
      cout << "present=" << hex << os.present << dec << endl;
      cout << "charging=" << hex << os.charging << dec << endl;
      cout << "discharging=" << hex << os.discharging << dec << endl;
      cout << "reserved=" << hex << os.reserved << dec << endl;
      cout << "powerPresent=" << hex << os.powerPresent << dec << endl;
      cout << "powerNG=" << hex << os.powerNG << dec << endl;
      cout << "inhibited=" << hex << os.inhibited << dec << endl;
      cout << "\n";
      for(int xx = 0; xx < 4; ++xx)
      {
        if(os.batRegFlag[xx][0xd])
          cout << "bat" << xx << " percent=" << os.batReg[xx][0xd] << "% ";
        if(os.batRegFlag[xx][0xa])
        {
          long tmp = os.batReg[xx][0xa];
          if(tmp & 0x8000)
            tmp = tmp - 65536;

          cout << " current=" << tmp << "mA";
        }

        cout << endl;
      }
      cout << "------------------------------\n";
    }
#endif
  }
}
