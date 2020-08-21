package org.firstinspires.ftc.teamcode.debugging;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.concurrent.Semaphore;

public class UdpServer implements Runnable {
    private static final int MAX_SEND_SIZE_PER_UPDATE = 600;

    private final int clientPort;
    private Semaphore sendLock;
    private boolean closed;

    private TimeProfiler timeProfiler;
    private long lastSendTime;

    private volatile String lastMessage;
    private volatile String currentMessage;

    public UdpServer(final int clientPort) {
        this.clientPort = clientPort;
        setSendLock(new Semaphore(1));
        setClosed(false);
        setTimeProfiler(new TimeProfiler(true));
        setLastMessage("");
        setCurrentMessage("");
    }

    public static int getMaxSendSizePerUpdate() {
        return MAX_SEND_SIZE_PER_UPDATE;
    }

    @Override
    public void run() {
        while(!isClosed()) {
            try {
                if(getTimeProfiler().getDeltaTime(TimeUnits.MILLISECONDS) < 50) {
                    continue;
                }

                getTimeProfiler().update(true);
                getSendLock().acquire();

                if(getCurrentMessage().length() > 0) {
                    splitMessageAndSend(getCurrentMessage());
                    setCurrentMessage("");
                } else if(getLastMessage().length() > 0) {
                    splitMessageAndSend(getLastMessage());
                    setLastMessage("");
                }

                getSendLock().release();
            } catch (final InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void splitMessageAndSend(final String message) {
        int startIndex = 0;
        int endIndex;
        do {
            endIndex = Range.clip(startIndex + getMaxSendSizePerUpdate(), 0, message.length() - 1);
            while(message.charAt(endIndex) != '%') {
                endIndex--;
            }

            sendUdpMessageRaw(message.substring(startIndex, endIndex + 1));
            startIndex = endIndex + 1;
        } while(endIndex != message.length() - 1);
    }

    private void sendUdpMessageRaw(final String message) {
        try(final DatagramSocket serverSocket = new DatagramSocket()) {
            final DatagramPacket datagramPacket =
                    new DatagramPacket(message.getBytes(), message.length(), InetAddress.getLocalHost(), getClientPort());
            serverSocket.send(datagramPacket);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addMessage(final String message) {
        if(!getSendLock().tryAcquire()) {
            setLastMessage(message);
        } else {
            setCurrentMessage(message);
            getSendLock().release();
        }
    }

    public int getClientPort() {
        return clientPort;
    }

    public Semaphore getSendLock() {
        return sendLock;
    }

    public void setSendLock(Semaphore sendLock) {
        this.sendLock = sendLock;
    }

    public long getLastSendTime() {
        return lastSendTime;
    }

    public void setLastSendTime(long lastSendTime) {
        this.lastSendTime = lastSendTime;
    }

    public boolean isClosed() {
        return closed;
    }

    public void setClosed(boolean closed) {
        this.closed = closed;
    }

    public void close() {
        setClosed(true);
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public String getLastMessage() {
        return lastMessage;
    }

    public void setLastMessage(String lastMessage) {
        this.lastMessage = lastMessage;
    }

    public String getCurrentMessage() {
        return currentMessage;
    }

    public void setCurrentMessage(String currentMessage) {
        this.currentMessage = currentMessage;
    }
}
