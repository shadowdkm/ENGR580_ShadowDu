    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 25;
            section.data(25)  = dumData; %prealloc

                    ;% rtP.A
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.B
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 100;

                    ;% rtP.Cob
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 130;

                    ;% rtP.DisturbanceVariance
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 200;

                    ;% rtP.K
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 210;

                    ;% rtP.Ki
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 240;

                    ;% rtP.Kp
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 261;

                    ;% rtP.L
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 282;

                    ;% rtP.ObservedVariance
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 352;

                    ;% rtP.TrackingEnabled
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 359;

                    ;% rtP.x0
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 360;

                    ;% rtP.RandomSource_MeanVal
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 370;

                    ;% rtP.Step_Time
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 371;

                    ;% rtP.Step_Y0
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 372;

                    ;% rtP.Step_YFinal
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 373;

                    ;% rtP.Step1_Time
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 374;

                    ;% rtP.Step1_Y0
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 375;

                    ;% rtP.Step1_YFinal
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 376;

                    ;% rtP.RandomSource_VarianceRTP
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 377;

                    ;% rtP.Integrator2_IC
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 378;

                    ;% rtP.Integrator1_IC
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 379;

                    ;% rtP.Constant1_Value
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 380;

                    ;% rtP.Constant2_Value
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 388;

                    ;% rtP.Constant3_Value
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 391;

                    ;% rtP.Constant4_Value
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 393;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 13;
            section.data(13)  = dumData; %prealloc

                    ;% rtB.ccu4dpkpbc
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.ahei004b0s
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 10;

                    ;% rtB.dw5qib4r4x
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 20;

                    ;% rtB.h43ikrnm2w
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 27;

                    ;% rtB.afd4wlreda
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 34;

                    ;% rtB.aqub0r2ekx
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 44;

                    ;% rtB.m3dx5xndf2
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 51;

                    ;% rtB.mqnbs0uqzk
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 52;

                    ;% rtB.c30cf505b4
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 59;

                    ;% rtB.lvo3gfwy02
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 69;

                    ;% rtB.g01h5zrukl
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 79;

                    ;% rtB.a21stht3j2
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 80;

                    ;% rtB.f2t000eoch
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 81;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 6;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 11;
            section.data(11)  = dumData; %prealloc

                    ;% rtDW.jmq3tsxawc.LoggedData
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.l5wjhqfbqc.LoggedData
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 2;

                    ;% rtDW.pegw3rnsv1.LoggedData
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.l0qhxzdoan.AQHandles
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 4;

                    ;% rtDW.jmxv5akigp.AQHandles
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 5;

                    ;% rtDW.dgtxfbydlw.AQHandles
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 6;

                    ;% rtDW.preosqjjml.AQHandles
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 7;

                    ;% rtDW.om1g1jtddz.AQHandles
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 8;

                    ;% rtDW.kg0cpsbr4n.AQHandles
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 9;

                    ;% rtDW.gnkcthcwfm.AQHandles
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 10;

                    ;% rtDW.otdgdsww4h.AQHandles
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.jkne1n5eru
                    section.data(1).logicalSrcIdx = 11;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.ou0nyrelia
                    section.data(2).logicalSrcIdx = 12;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.myhykfgvxr
                    section.data(1).logicalSrcIdx = 13;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.mapbnxxsya
                    section.data(2).logicalSrcIdx = 14;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.crw5jrqanh
                    section.data(1).logicalSrcIdx = 15;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.dubitlkqqo
                    section.data(2).logicalSrcIdx = 16;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.e2f2zvadqk
                    section.data(1).logicalSrcIdx = 17;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.lvor0bqhcv
                    section.data(2).logicalSrcIdx = 18;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.lw2jdaqrz3
                    section.data(1).logicalSrcIdx = 19;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.a2bz2wiwxl
                    section.data(2).logicalSrcIdx = 20;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 3091924031;
    targMap.checksum1 = 3399062413;
    targMap.checksum2 = 1667400322;
    targMap.checksum3 = 798752003;

