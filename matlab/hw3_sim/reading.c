function onJointPositionChange(uiHandle, id, newValue)
    joint_corres = Joints[id%10]
    if type(newValue)=='string' then
        if (tonumber(newValue) == nil or  newValue =='') then
            theta = 0
        else theta =  tonumber(newValue)/180*pi
        end
    else theta =  tonumber(newValue)/180*pi
    end
    
    sim.setJointTargetPosition(joint_corres, theta)   
end


function setMode(uiHandle, id)
    --print(id)
    mode = id
    if mode==101 then
        for i=1,6,1 do
            sim.setJointMode(Joints[i], sim.jointmode_force,0)
            simUI.setEnabled(uiHandle, i, true)
            simUI.setEnabled(uiHandle, i+10, true)
        end
    elseif mode==102 then
        for i=1,6,1 do
            sim.setJointMode(Joints[i], sim.jointmode_ik,1)
            simUI.setEnabled(uiHandle, i, false)
            simUI.setEnabled(uiHandle, i+10, false)
        end

    end
end

function sysCall_init()
    -- do some initialization here
    Joints={}
    for i=1,6,1 do
        Joints[i]=sim.getObjectHandle('Revolute_joint'..i)
    end
    tip = sim.getObjectHandle('tip')
    target = sim.getObjectHandle('target')
    base=sim.getObjectHandle('convex_0')
    ik = sim.getIkGroupHandle('IK_Group')
    ikFailedReportHandle=-1
        
    desiredConf={0,0,0,0,0,0} -- when in IK mode
    currentConf={0,0,0,0,0,0} -- when in IK mode
    initialTipPosRelative=sim.getObjectPosition(tip,base)
    m=sim.getObjectMatrix(tip,base)
    initialTipOrientRelative=sim.getEulerAnglesFromMatrix(m)
    
    mode = 101 --mode FK = 101, IK = 102
    
    pi = 3.1415926535897931
    
    UI = simUI.create('<ui enabled="true" modal="false" title="Indy7" resizable="true" closeable="true" layout="hbox" placement="relative" position="750,820" >'..
    '<group layout="vbox">'..
    '<radiobutton id="101" enabled="true" text="Foward Kinematics" checked="true" on-click="setMode"></radiobutton>'..
    '<radiobutton id="102" enabled="true" text="Inverse Kinematics" checked="false" on-click="setMode"></radiobutton>'..
    '</group>'..
    
    --1
    '<group>'..
    '<label enabled="true" text="Joint1(-180~180)"></label>'..
    '<label id= "21" enabled="true" text="0"> </label>'..
    '<hslider id="1" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="11" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
    
    --2
    '<group>'..
    '<label enabled="true" text="Joint2(-180~180)"></label>'..
    '<label id= "22" enabled="true" text="0"> </label>'..
    '<hslider id="2" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="12" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
    
    --3
    '<group>'..
    '<label enabled="true" text="Joint3(-180~180)"></label>'..
    '<label id= "23" enabled="true" text="0"> </label>'..
    '<hslider id="3" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="13" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
    
    --4
    '<group>'..
    '<label enabled="true" text="Joint4(-180~180)"></label>'..
    '<label id= "24" enabled="true" text="0"> </label>'..
    '<hslider id="4" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="14" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
    
    --5
    '<group>'..
    '<label enabled="true" text="Joint5(-180~180)"></label>'..
    '<label id= "25" enabled="true" text="0"> </label>'..
    '<hslider id="5" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="15" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
    
    --6
    '<group>'..
    '<label enabled="true" text="Joint6(-180~180)"></label>'..
    '<label id= "26" enabled="true" text="0"> </label>'..
    '<hslider id="6" enabled="true" minimum="-180" maximum="180" on-change="onJointPositionChange"></hslider>'..
    '<edit id="16" enabled="true" value="0" on-change="onJointPositionChange"> </edit>'..
    '</group>'..
--    '<text-browser id="31" enabled="false" text="x: 0, y: 0, z: 0"></text-browser>'..
    '</ui>'
    )
    --setMode(UI, 102)
end



function setUiJointPosition(uiHandle, i)
    theta = sim.getJointPosition(Joints[i])
    theta = theta/pi*180
    simUI.setSliderValue(uiHandle, i, tonumber(theta))
    simUI.setLabelText(uiHandle, i+20, tostring(string.format("%.4f",theta)))
end

function printTaskPosition(tip_or_target)
    if tip_or_target == 'tip' then object = tip
    else object = target
    end
    pos = sim.getObjectPosition(object, -1)
    ori = sim.getObjectOrientation(object, -1)
    m = sim.getObjectMatrix(object,base)
    m[12] = m[12] - 0.037
    for i=1,4 do
        if i==4 then table.insert(m, 1)
        else table.insert(m,0)
        end
    end
    T1 = {{0,-1,0,0},{1,0,0,0},{0,0,1,0},{0,0,0,1}}
    --T1 = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}
    T2 = {{0,1,0,0},{-1,0,0,0},{0,0,1,0},{0,0,0,1}}
    --T2 = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}
    tmp={}
    for i=1,4 do
        tmp[i] = {}
        for j=1,4 do
            tmp[i][j] = 0
            for k=1,4 do
                tmp[i][j] = tmp[i][j] + m[(i-1)*4+k]*T1[k][j]
            end
        end
    end
    
    mt = {}
    for i=1,4 do
        mt[i]  = {}
        for j=1,4 do
            mt[i][j] = 0
            for k=1,4 do
                mt[i][j] = mt[i][j] + T2[i][k]*tmp[k][j]
            end
        end
    end
    
    text = --"\n\a"..string.upper(tip_or_target).." Position"..
            --"\nx: "..string.format("%.4f",pos[1])..
            --"\ny: "..string.format("%.4f",pos[2])..
            --"\nz: "..string.format("%.4f",pos[3])..
            --"\n\a"..string.upper(tip_or_target).." Orientation(euler)"..
            --"\n"..string.format("%.4f",ori[1])..
            --"\n"..string.format("%.4f",ori[2])..
            --"\n"..string.format("%.4f",ori[3])
            "\n\a"..string.upper(tip_or_target).." Position"..
            "\nx: "..string.format("%.4f",mt[1][4])..
            "\ny: "..string.format("%.4f",mt[2][4])..
            "\nz: "..string.format("%.4f",mt[3][4])..
           
            "\n\a"..string.upper(tip_or_target).." Rotation matrix"..
            "\n"..string.format("%.4f",mt[1][1])
            ..string.format(" %.4f",mt[1][2])
            ..string.format(" %.4f",mt[1][3])..
            "\n"..string.format("%.4f",mt[2][1])
            ..string.format(" %.4f",mt[2][2])
            ..string.format(" %.4f",mt[2][3])..
            "\n"..string.format("%.4f",mt[3][1])
            ..string.format(" %.4f",mt[3][2])
            ..string.format(" %.4f",mt[3][3])
    print(text)
    --simUI.setText(UI, 21, text)
end

function sysCall_actuation()
    -- put your actuation code here
    for i=1,6,1 do
        setUiJointPosition(UI, i)
    end
    --printTaskPosition("target")
    printTaskPosition("tip")

    if mode==102 then
        if (sim.handleIkGroup(ik)==sim.ikresult_fail) then
            --print(sim.handleIkGroup(ik))
            --print('ik failed')
        end
    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
