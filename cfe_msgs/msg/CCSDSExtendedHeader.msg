# subsystem qualifier 
uint8[] subsystem
#  bits  shift   ------------ description ---------------- 
#  0x01FF   0  : Subsystem Id  mission defined              
#  0x0200   9  : Playback flag  0 = original, 1 = playback  
#  0x0400  10  : Endian:   Big = 0, Little (Intel) = 1      
#  0xF800  11  : EDS Version for packet definition used     
 
# system qualifier
uint8[] system_id  
#  0xFFFF   0  : System Id      mission defined             
