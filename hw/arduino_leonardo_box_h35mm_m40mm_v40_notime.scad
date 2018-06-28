
// Arduino Box with additional space for electronics
// Leonardo form  
//
// V3.1 - little bigger usb hole
// V4.0 reworked, V4.0a correction deckelweite
//
// (C)2013 Nischelwitzer Alexander nischelwitzer@gmail.com
// ----------------------------------------------------------- 

use<write/write.scad>

// ------------------------------------------- breadboard size

card_length = 69.0;
card_width  = 53.5;

// ------------------------------------------- 

box_do      =  0;  // 1 if additional box is used

box_height  = 35;
box_more    = 40;
shifter     = 10;

// ------------------------------------------- 

wall        =  2;
space       =  0.3;
dummy       =  1; // not used

deckel      = 12;

name_it     =  0;  // write some text
form_it     =  0;  // inside form for leonardo

// ------------------------------------------- 
do_light = 1; // print with lot of holes
do_easy = 1; // print w/out any support
// ################################################################
// ################################################################

// translate([0,card_width+shifter-10,0]) deckel();
translate([0,0,0])                     boxx();
translate([0,-(card_width+20),0])      deckel_innen();

// ################################################################
// ################################################################

module boxx() {
  difference() 
  {
    union()  
    {  
      color([1,0,0,1])  translate([0, 0, 0]) 
        cube([card_length+shifter,card_width+shifter,box_height]); 

      color("blue") translate([card_length+shifter-box_do*wall, 0, 0]) 
        cube([box_do*box_more,card_width+shifter,box_height]);
 	}

    translate([wall, wall, wall]) 
      cube([card_length-2*wall+shifter,card_width-2*wall+shifter,box_height]); 
    translate([card_length+shifter+wall-box_do*wall, wall, wall]) 
      cube([box_do*box_more-2*wall,card_width-2*wall+shifter,box_height]); 
   
   // usb connector
   if (do_easy != 1) { // will use support
       translate([-9,36,2]) cube([17,14,10]); 
   } else if (do_easy == 1) { // hole all way to the top to avoid support
       translate([-9,36,2]) cube([17,14,box_height+shifter]);
       }
   

   // connector hole inside
   if (box_do == 1)
   {
     translate([card_length,(card_width+shifter)/2,box_height-deckel/2]) 
       rotate([0,90,0]) cylinder(r=17,h=20);  
     translate([card_length,wall,box_height-deckel-2]) 
       cube([20,card_width+shifter-2*wall,deckel*2]);
   }

   // connector hole outside
   for (a =[8,5,2]){ // repeat couple of times to get nice transition
       translate([card_length+box_do*box_more,(card_width+shifter)/2,box_height-a]) 
     rotate([0,90,0]) cylinder(r=10,h=20); 
       }
    
   // pefuse with holes where possible
   if (do_light == 1) {
       // Add rectangular "windows" on sides
       c_x = 30;
       c_y = 1/3 * c_x;
       c_z = c_x;
       translate([card_length/2-c_x/2 + shifter/2,-c_y/2,box_height/2-c_z/4]) cube([c_x,c_y,c_z]);
       translate([card_length/2-c_x/2 + shifter/2,card_width+shifter/2-c_y/2 + shifter/2,box_height/2-c_z/4]) cube([c_x,c_y,c_z]);
       
       // Add same connector hole also on front side
       if (do_easy != 1){ // Don't over do it and avoid collision of shapes
           for (a =[8,5,2]){
           translate([-shifter,(card_width+shifter)/2,box_height-a])
           rotate([0,90,0]) cylinder(r=10,h=20); 
           }
       }
   
       // logo open hardware
       translate([(card_length+box_do*box_more)/2+shifter/2,(card_width+shifter/2)/2, -10]) rotate([0,0,0]) 
       linear_extrude(height=20) import(file="./oshw_logo_2d.dxf", scale=0.25);
   }
   }
  if (form_it == 1) // Include board holder
  translate([card_length/2+shifter/2,card_width/2+shifter/2,wall])  
    import("bumper.stl");
}


// ################################################################

module deckel() {
   // not used
  difference() 
  {
    union()  
    {   
      color([0,1,0,1])  translate([0-wall-space, shifter*2-wall-space, 0]) 
        cube([card_length+shifter+box_do*box_more+2*(wall+space)-box_do*wall,
              card_width+shifter+2*(wall+space),deckel]); 
	}

    translate([0-space, shifter*2-space, wall]) 
      cube([card_length+shifter+box_do*box_more+space*2-box_do*wall,
            card_width+shifter+space*2,deckel]); 
  }

  if (name_it == 1) translate([wall*4, wall*4+shifter*2, wall]) rotate([0,0,0]) 
    write("Nischelwitzer",t=2,h=8.0,center=false,font="letters.dxf");
}

// ################################################################

module deckel_innen() 
{
  difference() 
  {
    union()  
    {   
      color([0,1,1,1]) translate([0, 0, 0]) 
      cube([card_length+shifter+box_do*box_more-box_do*wall,
            card_width+shifter,wall]); 

      color([0,0,1,1]) translate([wall+space/2,wall+space/2, 0]) 
      cube([card_length+shifter+box_do*box_more-space-2*wall-box_do*wall,
           card_width+shifter-space-2*wall,deckel]);
    
    // Create overlap from top to hide some of insides
    if (do_easy==1){
        mltplier = 1.2;
        color([0,0,1,1]) translate([wall+space/2,card_width+-(1-mltplier)*14-36-space-2*wall,2]) cube([wall+2.5*space,mltplier*14,2*wall+deckel+box_height/2-10]);
        }
	}
    // Create rim by substracting inner cube
    translate([2.5*wall, 2.5*wall, wall])
      cube([card_length+shifter+box_do*box_more-5.0*wall-box_do*wall,
            card_width+shifter-5*wall, deckel]);
    
    // logo open hardware
    // pos mitte
    translate([(card_length+box_do*box_more)/2+shifter/2, (card_width+shifter/2+shifter*name_it)/2, -10]) rotate([0,0,0]) 
    linear_extrude(height=20) import(file="./oshw_logo_2d.dxf", scale=0.25);
    
    // pos over leonardo
    // translate([(card_length+box_do*box_more)/2+shifter/2-30, 0, -10]) rotate([0,0,0]) 
    //   linear_extrude(height=20) import(file="./oshw_logo_2d.dxf", scale=0.3);
  }

  if (name_it == 1) // write name on the inside of the deckel
  translate([wall*4, wall*4, wall]) rotate([0,0,0]) 
    write("milesahead",t=2,h=9.0,center=false,font="letters.dxf");
  // outside would be also nice but difficult to print
}
// ################################################################