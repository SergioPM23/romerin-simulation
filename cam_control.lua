function sysCall_init()
    -- Obtener identificadores relativos a este script (que esta colgado del body)
    motor1 = sim.getObject('../motor1_cam')
    motor2 = sim.getObject('../motor2_cam')

    -- Crear interfaz de usuario con sliders
    xml = [[
    <ui title="Control de la camara" closeable="true" resizeable="false" layout="vbox" placement="relative" position="500,400">
        <label text="Giro vertical de la camara (motor1_cam)"/>
        <hslider minimum="-180" maximum="180" on-change="onSlider1Change" id="1"/>
        <label text="Giro horizontal de la camara (motor2_cam)"/>
        <hslider minimum="-60" maximum="60" on-change="onSlider2Change" id="2"/>
    </ui>
    ]]
    ui = simUI.create(xml)
end

-- Convertir grados a radianes y enviar a las articulaciones
function onSlider1Change(ui, id, newVal)
    sim.setJointTargetPosition(motor1, math.rad(newVal))
end

function onSlider2Change(ui, id, newVal)
    sim.setJointTargetPosition(motor2, math.rad(newVal))
end

function sysCall_cleanup()
    if ui then
        simUI.destroy(ui)
    end
end
