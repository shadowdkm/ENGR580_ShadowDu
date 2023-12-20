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
            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% rtP.A
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.B
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 100;

                    ;% rtP.K
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 130;

                    ;% rtP.stateNoiseGain
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 160;

                    ;% rtP.stepVal
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 170;

                    ;% rtP.x0
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 173;

                    ;% rtP.RandomSource1_MeanVal
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 183;

                    ;% rtP.CheckStaticUpperBound_max
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 184;

                    ;% rtP.RandomSource1_VarianceRTP
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 185;

                    ;% rtP.Step_Time
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 186;

                    ;% rtP.Step_Y0
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 187;

                    ;% rtP.Constant_Value
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 188;

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
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% rtB.kmwcbilhqq
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.ku2bcekbpk
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 10;

                    ;% rtB.kgojaz3q5t
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 20;

                    ;% rtB.i3et0lx2gv
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 23;

                    ;% rtB.fx25dj0plx
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 26;

                    ;% rtB.b3g5ksvk55
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 29;

                    ;% rtB.gp3f1tfouj
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 39;

                    ;% rtB.h1cjs4ulia
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 40;

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
            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% rtDW.pgelvgrvt1.LoggedData
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.bildbdyfzk.AQHandles
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 2;

                    ;% rtDW.h3mcukzweq.AQHandles
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.fy4dfotmyf.AQHandles
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 4;

                    ;% rtDW.o4yowjfnvg.AQHandles
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 5;

                    ;% rtDW.aw1sjpengb.AQHandles
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.o3aryxf3r0
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.atzgtyu2uw
                    section.data(1).logicalSrcIdx = 7;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.bgwp0luil4
                    section.data(2).logicalSrcIdx = 8;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.prcj0bragy
                    section.data(1).logicalSrcIdx = 9;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.e3nps1yxro
                    section.data(1).logicalSrcIdx = 10;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.hwccjcyk2m
                    section.data(1).logicalSrcIdx = 11;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.maae0yta5q
                    section.data(2).logicalSrcIdx = 12;
                    section.data(2).dtTransOffset = 10;

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


    targMap.checksum0 = 1876633778;
    targMap.checksum1 = 710715817;
    targMap.checksum2 = 2342842469;
    targMap.checksum3 = 1004280244;

