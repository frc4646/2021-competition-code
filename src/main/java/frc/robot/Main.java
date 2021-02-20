/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
                              ..                    ..                         ...               
           .,.         ',,,,,,,;;:;;;;;;;;;;;;;;;;;;;;:::;;;;:;;;;;;;;;;;;;;;;;;;;;;:;;,.           
           ;lc.       .:ooollllllllllllllllllllllllllooooollllllllllllllllllllllllllllll:.          
          ,loo:.      .:ool;........................,coool,..........''''''''''''''''''''.          
         ,loooo:.     .:ooc,.                     ..;loool:.        .';:::::::::::::::::,.          
        'lol::ol;.   .'cooc,.                    .';loc:col;.        ,looolllllllllllooo:.          
       .col, .col;.   ':ool;.............       .;clol,.,col;.       ,looc,''''''''';loo:.          
      .col;.  .col;.  'cooollllllllllllc'      .,lool;. .,lol;.      ,loo:.         'coo:.          
     .coo:.    'col,...,::::;;::::::clol,     .,cool:.   .;lol,.    .,loo:.         'loo:.          
    .:ool:,,,',;cool,.             .;lol,     'coool:;;;;;:llol,.    ,loo:.         'coo:.          
   .:oolllllllllllloc'             .;lol,    'coollllllllllllooc,.   ,coo:.         'coo:.          
  .;loc'..........;loc'.           .;lol,   .cooc'..........':ooc'  .,coo:.         'coo:.          
 .;loc'           .;loc:::::::::::::clol,  .:ooc'            .cooc. .,cool;,,,,,,,,,:loo:.          
 ,llc'             .;looolllllllllllllll,..;llc'              'cll:..,looollllllllllllll:.          
 ....               .;looc,.............. .....                ......,looc'..............           
                      'col:'.                                       .,loo:.                         
                       .;lol:'.                                      ,loo:.                   .     
                         .;lolc;..                                   ,loo:.               ....      
                           .,:lolc;'.                                ,loo:.           ......        
                              .';clll:;,..                           ,loo:.     ...'''...           
                                 ..';:lllc:;;,'.....                .,looc'.',,,,'..                
                                      ..',;:cllllccc::;;;;;;,,,,;;;;;clool;,'..                     
                                             ....''',,,;;;;;;;;;,,''.;loo:.                         
                                                                     .',,.                          
*/

/* Corgo                                                                                               
                      yh/       -yy                                                                 
                      mNmy`    omNN:                                                                
                      dNNNh: `sNNNNs                                                                
                      sNNNNmddNNNNNm.                                                               
                      .NNNNNNNNNNNNNy`                                                              
                      .NNNNNNNNNNNNNNo                                                              
                      :NNNNNNNNNNNNNNm/                                                             
                    .omNNNNNNNNNNNNNNNm-                                                            
                   .dNNNNNNNNNNNNNNNNNNd.                                                           
                   `+dNNNNNNNNNNNNNNNNNNd:                                                          
                     `-//hNNNNNNNNNNNNNNNmh+:-.``                                                   
                         mNNNNNNNNNNNNNNNNNNNmddhyo++++++///++++++/:--..`                           
                         mNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNmmmdhy+-                        
                         hNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNmy`                      
                         oNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNy.                     
                         /NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNd+`                   
                         -NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNs                   
                          yNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNd                   
                          `+dNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNd                   
                            .odNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN/                   
                              .+dNNNNNNNNNNNNNNNNNNNNNNNNNNmmmmNNNNNNNNNNNNNNNNm`                   
                                :NNNNNNNNNmddddddmmmmdds+/::---:/+hmNNNNNmNNNNNo                    
                                 hNNNNNNmy.``````.-..`             -/dNNN++dNNNs                    
                               `.hNNNNNm/                          `.mNNd. :NNNm.                   
                              /hmmNNNNm/                         `shdmmo``:sNNm+                    
                                `/oo++-                           `--:.  sdmdd:                     
                                                                           `                                                                                                        
*/


package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
