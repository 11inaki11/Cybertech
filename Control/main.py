import utime

if (utime.ticks_diff(utime.ticks_ms(), last_time) > 1000):
    machine.disable_irq()
    rpmI = ((contadorI - prevcontadorI) / 20) * 60
    prevcontadorI = contadorI
    prevcontadorD = contadorD
    machine.enable_irq()
    last_time = utime.ticks_ms()
    print("RPM Izquierdo: ", rpmI)
