sim = require'sim'
simIK = require'simIK'

-- Parametros generales
body = sim.getObject(string.format('..'))
romerinPathRef = sim.getObject(string.format('../RomerinPathRef'))
bodyMass = sim.getObjectFloatParam(body, sim.shapefloatparam_mass)

-- Estructuras para manejar articulaciones y enlaces
links = {}
joints = {}
masses = {0.212, 0.443, 0.627, 0.107, 0.112, 0.26}
legMass = 1.761

-- Variables de cinematica inversa (IK)
legs = {}
suctionHandles = {}
tips = {}
targets = {}
ikGroups = {}
suctionFlags = {}
movementCompleted = true
ikEnv = nil

-- Parametros de movimiento
movementIncrement = 0.1
activeLeg = 1

-- Torques maximos por motor
maxTorques = {
    {9.9, 6.4, 6.4, 6.4, 0.39, 0.39},
    {9.9, 6.4, 6.4, 6.4, 0.39, 0.39},
    {9.9, 6.4, 6.4, 6.4, 0.39, 0.39},
    {9.9, 6.4, 6.4, 6.4, 0.39, 0.39}
}

-- Variable global para controlar el temporizador inicial
startWaitTime = nil
waitDuration = 0.25 -- Tiempo de espera inicial en segundos

-- Fases del movimiento
currentPhase = "SUBIDA"
liftHeight = 0.1  -- Altura minima de elevacion
movementThreshold = 0.05 -- Umbral de distancia para cambio de fase

-- Variables para identificadores del COM
COM = nil
COM0 = nil
-- Variable de posicion del COM respecto a PathRef
COM_pos = {}
-- Variable de posicion inicial del COM respecto al PathRef
COM0_pos = {}

-- Variable para identificadores de los sensores auxiliares
auxSensors = {
    [1] = {},
    [2] = {},
    [3] = {},
    [4] = {}
}

function calculateCOM()
    local totalMass = bodyMass
    local totalCOM = {0, 0, 0}

    -- Agregar contribucion del cuerpo central
    local bodyCOM = sim.getObjectPosition(body, romerinPathRef)
    
    for j = 1, 3 do
        totalCOM[j] = bodyCOM[j] * bodyMass
    end
    
    -- Agregar contribucion de patas
    for l = 1, 4 do
        for i = 1, #links[l] do
            if sim.isHandleValid(links[l][i]) == 1 then
                local pos = sim.getObjectPosition(links[l][i], romerinPathRef)
                for j = 1, 3 do
                    totalCOM[j] = totalCOM[j] + pos[j] * masses[i]
                end
                totalMass = totalMass + masses[i]
            end
        end
    end
    
    -- Calcular el centro de masa total
    for j = 1, 3 do
        totalCOM[j] = totalCOM[j] / totalMass
    end
    
    return totalCOM
end


function setRest()
    -- Configurar el estado de reposo
    for leg = 1, 4 do -- Fijo a 4 patas
        -- Activar bandera de adhesion para todas las ventosas
        suctionFlags[leg] = true
    end
    -- Adherir ventosas
    suction_control()
    
    -- Bloquear las articulaciones 4 en 0 grados, 5 y 6 en su posicion actual
    for leg = 1, 4 do
        for i = 4, 6 do
            local joint = legs[leg].joints[i]
            if i == 4 then
                sim.setJointInterval(joint, false, {0, 0}) -- Bloquear en 0 grados
            else
                keepSuctionCupParallel()
                local currentPosition = sim.getJointPosition(joint)
                sim.setJointInterval(joint, false, {currentPosition, 0}) -- Bloquear en la posicion actual
            end
        end
    end
end

function suction_control()
    -- Ahesion de las ventosas al suelo
    for leg = 1, 4 do
        local sc = suctionHandles[leg]
        local dummyParent = sim.getObjectParent(sc.dummy)
        if suctionFlags[leg] then
            if dummyParent == sc.parent then
                local index = 0
                while true do
                    local shape = sim.getObjects(index, sim.object_shape_type)
                    if shape == -1 then break end
                    if sim.getObjectInt32Param(shape, sim.shapeintparam_respondable) ~= 0 and sim.checkProximitySensor(sc.sensor, shape) == 1 then
                        sim.setObjectParent(sc.dummy, sc.parent, true)
                        local matrix = sim.getObjectMatrix(sc.dummy2)
                        sim.setObjectMatrix(sc.dummy, matrix)
                        sim.setObjectParent(sc.dummy, shape, true)
                        sim.setLinkDummy(sc.dummy, sc.dummy2)
                        
                        -- Agregar fuerza de adhesion en sensor central
                        local sensorPos = sim.getObjectPosition(sc.sensor, sc.parent)
                        sim.addForce(sc.parent, sensorPos, {0, 0, -200})
                        break
                    end
                    index = index + 1
                end
            end
        else
            -- Quitar fuerza de adhesion en sensor central
            local sensorPos = sim.getObjectPosition(sc.sensor, sc.parent)
            sim.addForce(sc.parent, sensorPos, {0, 0, 0})
            
            -- Desadhesion de las ventosas al suelo
            if dummyParent ~= sc.parent then
                sim.setLinkDummy(sc.dummy, -1)
                sim.setObjectParent(sc.dummy, sc.parent, true)
                local matrix = sim.getObjectMatrix(sc.dummy2)
                sim.setObjectMatrix(sc.dummy, matrix)
            end
        end
    end
    
    for leg = 1, 4 do
        local sc = suctionHandles[leg]
        if suctionFlags[leg] then
            for i = 1, 8 do
                local sensor = auxSensors[leg][i]
                if sensor ~= -1 then
                    local detected = false
                    local index = 0
                    while true do
                        local shape = sim.getObjects(index, sim.object_shape_type)
                        if shape == -1 then break end
                        if sim.getObjectInt32Param(shape, sim.shapeintparam_respondable) ~= 0 and sim.checkProximitySensor(sensor, shape) == 1 then
                            detected = true
                            break
                        end
                        index = index + 1
                    end
                    if detected then
                        -- Agregar fuerza de adhesion en sensor auxiliar
                        local relPos = sim.getObjectPosition(sensor, sc.parent)
                        sim.addForce(sc.parent, relPos, {0, 0, -200})
                    end
                end
            end
        else
            for i = 1, 8 do
                local sensor = auxSensors[leg][i]
                if sensor ~= -1 then
                    -- Quitar fuerza de adhesion en sensor auxiliar
                    local relPos = sim.getObjectPosition(sensor, sc.parent)
                    sim.addForce(sc.parent, relPos, {0, 0, 0})
                end
            end
        end
    end
end

function unlockJoints(leg)
    -- Desbloquear las articulaciones 4, 5 y 6 respetando sus limites en radianes
    local jointLimits = {
        {math.rad(-180), math.rad(360)}, -- Articulacion 4: [-180, 180]
        {math.rad(-90), math.rad(180)},  -- Articulacion 5: [-90, 90]
        {math.rad(-180), math.rad(360)}  -- Articulacion 6: [-180, 180]
    }
    for i = 4, 6 do
        local joint = legs[leg].joints[i]
        local minValue = jointLimits[i - 3][1]
        local range = jointLimits[i - 3][2]
        sim.setJointInterval(joint, false, {minValue, range}) -- Desbloquear articulaciones
    end
end

function keepSuctionCupParallel()
    for leg = 1, 4 do
        local joints = legs[leg].joints
        local motor2_pos = sim.getJointPosition(joints[2])
        local motor3_pos = sim.getJointPosition(joints[3])
        -- Calculo preciso del angulo necesario
        local motor5_pos = -math.pi / 2 + motor3_pos - motor2_pos
        -- Asegurar que el valor asignado no introduce errores de redondeo
        sim.setJointPosition(joints[5], motor5_pos)
        -- Actualizar la cinematica inversa
        simIK.handleGroup(ikEnv, ikGroups[leg], {syncWorlds = true, allowError = true})
    end
end


-- Configuracion inicial
function sysCall_init()
    -- Configuracion de cinematica inversa (IK)
    ikEnv = simIK.createEnvironment()
    for leg = 1, 4 do
        legs[leg] = {joints = {}}
        for joint = 1, 6 do
            legs[leg].joints[joint] = sim.getObject(string.format('../motor%d_l%d', joint, leg))
        end

        suctionHandles[leg] = {
            sensor = sim.getObject(string.format('../Sensor_l%d', leg)),
            dummy = sim.getObject(string.format('../LoopClosureDummy1_l%d', leg)),
            dummy2 = sim.getObject(string.format('../LoopClosureDummy2_l%d', leg)),
            parent = sim.getObject(string.format('../link7_l%d_pure', leg))
        }
        
        for i = 1, 8 do 
            auxSensors[leg][i] = sim.getObject(string.format("../auxSensor%d_l%d", i, leg))
        end

        tips[leg] = sim.getObject(string.format('../Tip_l%d', leg))
        targets[leg] = sim.getObject(string.format('../Target_l%d', leg))
        ikGroups[leg] = simIK.createGroup(ikEnv)
        --simIK.setGroupCalculation(ikEnv, ikGroups[leg], simIK.method_pseudo_inverse, 0.1, 100)
        simIK.setGroupCalculation(ikEnv, ikGroups[leg], simIK.method_damped_least_squares, 0, 100)
        simIK.addElementFromScene(ikEnv, ikGroups[leg], body, tips[leg], targets[leg], simIK.constraint_position)
    end
    
    -- Configuracion para la compensacion gravitacional
    for leg = 1, 4 do
        links[leg] = {}
        joints[leg] = {}
        for link = 2, 7 do
            local linkName = string.format('../link%d_l%d_pure', link, leg)
            links[leg][link - 1] = sim.getObject(linkName)
        end
        for joint = 1, 6 do
            local jointName = string.format('../motor%d_l%d', joint, leg)
            joints[leg][joint] = sim.getObject(jointName)
        end
    end
    
    -- Adherir ventosas y bloquear articulaciones 5 y 6 de todas las patas
    setRest()
    -- Configuracion inicial para activar el temporizador
    startWaitTime = sim.getSimulationTime() -- Tiempo inicial
    
    -- Identificadores del COM (0 = COM inicial)
    COM = sim.getObject(string.format('../COM'))
    COM0 = sim.getObject(string.format('../COM_0'))
    -- Poisicion incial del COM
    COM0_pos = calculateCOM()
end
        
function sysCall_actuation()
    print(activeLeg, currentPhase)
    -- Posicion inicial del COM
    sim.setObjectPosition(COM0, COM0_pos, romerinPathRef)
    
    -- Actualizar posicion del COM
    COM_pos = calculateCOM()
    sim.setObjectPosition(COM, COM_pos, romerinPathRef)
    
    -- Comprobar si el tiempo de espera inicial ha terminado
    if startWaitTime ~= nil then
        local currentTime = sim.getSimulationTime()
        if currentTime - startWaitTime < waitDuration then
            return -- Salir de sysCall_actuation hasta que se cumpla el tiempo de espera
        else
            startWaitTime = nil -- Desactivar el temporizador inicial
        end
    end
    
    -- Control del movimiento de la pata activa
    if movementCompleted then        
        local leg = activeLeg
        local targetPose = sim.getObjectPose(targets[leg], romerinPathRef)
        local tipPose = sim.getObjectPose(tips[leg], romerinPathRef)
        
        if currentPhase == "SUBIDA" then
            suctionFlags[leg] = false
            suction_control()
            unlockJoints(leg)
            targetPose[3] = tipPose[3] + liftHeight
            sim.setObjectPose(targets[leg], targetPose, romerinPathRef)
            simIK.handleGroup(ikEnv, ikGroups[leg], {syncWorlds = true, allowError = true})
            movementCompleted = false
        elseif currentPhase == "MOVIMIENTO" then
            targetPose[2] = tipPose[2] + 0.05 * movementIncrement
            sim.setObjectPose(targets[leg], targetPose, romerinPathRef)
            simIK.handleGroup(ikEnv, ikGroups[leg], {syncWorlds = true, allowError = true})            
            movementCompleted = false
        elseif currentPhase == "BAJADA" then
            targetPose[3] = 0
            sim.setObjectPose(targets[leg], targetPose, romerinPathRef)
            simIK.handleGroup(ikEnv, ikGroups[leg], {syncWorlds = true, allowError = true})
            movementCompleted = false
        end
    else
        local leg = activeLeg
        local targetPose = sim.getObjectPose(targets[leg], romerinPathRef)
        if currentPhase == "SUBIDA" and math.abs(sim.getObjectPose(tips[leg], romerinPathRef)[3] - targetPose[3]) < movementThreshold then
            currentPhase = "MOVIMIENTO"
            movementCompleted = true
        elseif currentPhase == "MOVIMIENTO" and math.abs(sim.getObjectPose(tips[leg], romerinPathRef)[2] - targetPose[2]) < movementThreshold / 10 then
            currentPhase = "BAJADA"
            movementCompleted = true
        elseif currentPhase == "BAJADA" and math.abs(sim.getObjectPose(tips[leg], romerinPathRef)[3] - targetPose[3]) < 0.0035 then
            -- Mover el cuerpo de forma progresiva antes de setRest()
            local bodyPose = sim.getObjectPose(body, romerinPathRef)
            bodyPose[2] = bodyPose[2] + movementIncrement * 0.5 -- Coeficiente de 0.5 para suavizar el movimiento
            sim.setObjectPose(body, bodyPose, romerinPathRef)
            simIK.handleGroup(ikEnv, ikGroups[leg], {syncWorlds = true, allowError = true})
            currentPhase = "SUBIDA"
            movementCompleted = true
            setRest()
            activeLeg = (activeLeg % 4) + 1
        end
    end
end

function sysCall_sensing()
    keepSuctionCupParallel()
    local gravity = sim.getArrayParameter(sim.arrayparam_gravity)
    local bodyPosition = sim.getObjectPosition(body, sim.handle_world)
    
    local deltaX = COM_pos[1] - COM0_pos[1]
    local deltaY = COM_pos[2] - COM0_pos[2]
    local deltaZ = COM_pos[3] - COM0_pos[3]

    -- Fuerza correctiva por desviacion del COM
    local correctionFactor = 2700
    local f_corrX = -correctionFactor * deltaX
    local f_corrY = -correctionFactor * deltaY
    local f_corrZ = -correctionFactor * deltaZ

    -- Masa total del cuerpo + una pata (constante)
    local totalMass = bodyMass + legMass
    local forceFactor = 1

    -- Fuerza para compensar el peso (cuerpo central + pata en movimiento)
    local f_gravX = -forceFactor * gravity[1] * totalMass
    local f_gravY = -forceFactor * gravity[2] * totalMass
    local f_gravZ = -forceFactor * gravity[3] * totalMass

    local totalFx = f_corrX + f_gravX
    local totalFy = f_corrY + f_gravY
    local totalFz = f_corrZ + f_gravZ

    sim.addForce(body, {0, 0, 0}, {totalFx, totalFy, totalFz})
    
    -- Mantener romerinPathRef en x = 0 (movimiento rectilineo) y z = 0 constantemente
    local refPosition = sim.getObjectPosition(romerinPathRef, sim.handle_world)
    refPosition[1] = 0  -- Fijar en x = 0
    refPosition[3] = 0  -- Fijar en z = 0
    sim.setObjectPosition(romerinPathRef, refPosition, sim.handle_world)
end

-- sysCall_cleanup: Restaurar TODO
function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
    for leg = 1, 4 do
        for joint = 1, 6 do
            -- Restaurar los torques maximos originales al finalizar
            sim.setJointMaxForce(joints[leg][joint], maxTorques[leg][joint])
        end
        suctionFlags[leg] = true
    end
    -- Desadhesion de todas las ventosas
    for leg = 1, 4 do
        local sc = suctionHandles[leg]
        sim.setLinkDummy(sc.dummy, -1)
        sim.setObjectParent(sc.dummy, sc.parent, true)
        local matrix = sim.getObjectMatrix(sc.dummy2)
        sim.setObjectMatrix(sc.dummy, matrix)
    end

    -- Restablecer posiciones iniciales y limites de las articulaciones
    local initialPositions = {0, 80, 145, 0, -25, 0} -- Valores en grados
    local jointLimits = { -- Limites: {min, rango} en grados
        {-110, 220}, -- Motor 1
        {-115, 219}, -- Motor 2
        {-55, 215},  -- Motor 3
        {-180, 360}, -- Motor 4
        {-90, 180},  -- Motor 5
        {-180, 360}  -- Motor 6
    }

    local tolerance = math.rad(0.001) -- Tolerancia para la precision (en radianes)

    for leg = 1, 4 do
        for i = 1, 6 do
            local joint = legs[leg].joints[i]

            -- Temporalmente eliminar restricciones de movimiento
            sim.setJointInterval(joint, false, {math.rad(-180), math.rad(360)})

            -- Iterar hasta alcanzar la posicion deseada con precision
            local targetPosition = math.rad(initialPositions[i])
            while true do
                sim.setJointPosition(joint, targetPosition)
                local currentPosition = sim.getJointPosition(joint)
                if math.abs(currentPosition - targetPosition) < tolerance then
                    break -- Salir del bucle cuando se alcance la precision
                end
            end

            -- Restablecer los limites y rangos angulares definitivos
            local minValue = math.rad(jointLimits[i][1])
            local range = math.rad(jointLimits[i][2])
            sim.setJointInterval(joint, false, {minValue, range})
        end
    end
    
    -- Posicionar los Tip_lx y Target_lx
    for leg = 1, 4 do
        local tip = tips[leg]
        local target = targets[leg]
        local ventosa = suctionHandles[leg].parent -- link7_lx_pure
        local dummy1 = suctionHandles[leg].dummy
        local dummy2 = suctionHandles[leg].dummy2

        -- Obtener posicion de la ventosa
        local ventosaPose = sim.getObjectPosition(ventosa, sim.handle_world)

        -- Posicionar Tip_lx
        sim.setObjectPosition(tip, -1, {ventosaPose[1], ventosaPose[2], 0})

        -- Posicionar Target_lx igual al Tip_lx
        sim.setObjectPosition(target, {ventosaPose[1], ventosaPose[2], 0}, sim.handle_world)
        sim.setObjectPosition(dummy1, {ventosaPose[1], ventosaPose[2], 0}, sim.handle_world)
        sim.setObjectPosition(dummy2, {ventosaPose[1], ventosaPose[2], 0}, sim.handle_world)
    end

    -- Posicionar RomerinPathRef respecto al body
    local bodyPose = sim.getObjectPosition(body, sim.handle_world)
    sim.setObjectPosition(romerinPathRef, {bodyPose[1], bodyPose[2], 0}, sim.handle_world)
end
