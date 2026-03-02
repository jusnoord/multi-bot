package frc.robot.util;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * creates a fake a driver station to enable/disable the slave robot
 */
public class DSSim {
    private interface ByteModifer {
        void modify(byte[] data, short sendcount);
    }

    private Thread m_thread;
    private short sendCount = 0;
    private boolean usingFake = false;
    public boolean isEnabled = false;

    private void generateEnabledDsPacket(byte[] data, short sendCount) {
        data[0] = (byte) (sendCount >> 8);
        data[1] = (byte) sendCount;
        data[2] = 0x01; // general data tag
        data[3] = 0x04; // teleop enabled
        data[4] = 0x10; // normal data request
        data[5] = 0x00; // red 1 station
    }

    private void generateDisabledDsPacket(byte[] data, short sendCount) {
        generateEnabledDsPacket(data, sendCount);
        data[3] = 0x03; // teleop disabled
    }

    private void start(ByteModifer modifier) {
        m_thread = new Thread(() -> {
            DatagramSocket socket;
            try {
                socket = new DatagramSocket();
            } catch (SocketException e1) {
                e1.printStackTrace();
                return;
            }

            InetSocketAddress addr = new InetSocketAddress("127.0.0.1", 1110);
            byte[] sendData = new byte[6];
            DatagramPacket packet = new DatagramPacket(sendData, 0, 6, addr);

            int initCount = 0;
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    Thread.sleep(20);
                    modifier.modify(sendData, sendCount++);
                    if (initCount < 50) {
                        initCount++;
                        sendData[3] = 0;
                    }
                    packet.setData(sendData);
                    socket.send(packet);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                } catch (IOException ex) {
                    ex.printStackTrace();
                }
            }
            socket.close();
        });

        m_thread.start();
    }

    private void stop() {
        if (m_thread == null) {
            return;
        }

        m_thread.interrupt();

        try {
            m_thread.join(1000);
            m_thread = null;
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    private boolean checkRealDS() {
        return !usingFake && m_thread == null;
    }

    public boolean isFakeDS() {
        return m_thread != null;
    }

    /**
     * initializes the DSSim, should be called once at the start of the robot code
     */
    public void init() {
        // usingFake = !DriverStation.isDSAttached();
        usingFake = true;
        // if (usingFake) {
        //     disable();
        // }
    }

    /**
     * enables the DSSim, should be called when the robot is enabled
     */
    public void enable() {
        stop();
        // if (checkRealDS()) {
        //     return;
        // }

        isEnabled = true;
        start(this::generateEnabledDsPacket);
    }

    /*disables the DSim, should be called when the robot is disabled */
    public void disable() {
        stop();
        // if (checkRealDS()) {
        //     return;
        // }

        isEnabled = false;
        start(this::generateDisabledDsPacket);
    }

    /**
     * returns a command that disables the DSSim
     * @return a command that disables the DSSim
     */
    public Command disableCmd() {
        return new InstantCommand(this::disable).ignoringDisable(true);
    }
}
