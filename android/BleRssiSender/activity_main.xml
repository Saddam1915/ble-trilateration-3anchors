package com.example.blerssisender

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import kotlin.concurrent.thread

class MainActivity : AppCompatActivity() {

    private val REQUEST_PERMISSION_CODE = 1
    private lateinit var bluetoothLeScanner: BluetoothLeScanner
    private var scanning = false
    private var sending = false  // Flag para envío en tiempo real

    private val SERVER_IP = "10.0.7.225" // Cambia por IP de tu PC
    private val SERVER_PORT = 5005

    private lateinit var tvStatus: TextView
    private lateinit var btnScan: Button
    private lateinit var btnSend: Button
    private lateinit var rvDevices: RecyclerView
    private val deviceList = mutableListOf<Device>()
    private lateinit var adapter: DeviceAdapter

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvStatus = findViewById(R.id.tv_status)
        btnScan = findViewById(R.id.btn_scan)
        btnSend = findViewById(R.id.btn_send)
        rvDevices = findViewById(R.id.rv_devices)

        adapter = DeviceAdapter(deviceList)
        rvDevices.layoutManager = LinearLayoutManager(this)
        rvDevices.adapter = adapter

        btnScan.setOnClickListener {
            if (!scanning) {
                if (hasPermissions()) startBleScan() else requestPermissions()
            } else {
                stopBleScan()
            }
        }

        btnSend.setOnClickListener {
            sending = !sending
            updateStatus()
            if (sending) {
                Toast.makeText(this, "Envío UDP en tiempo real ACTIVADO", Toast.LENGTH_SHORT).show()
                if (!scanning && hasPermissions()) {
                    startBleScan()
                }
            } else {
                Toast.makeText(this, "Envío UDP DETENIDO", Toast.LENGTH_SHORT).show()
            }
        }

        if (!hasPermissions()) {
            requestPermissions()
        }
        updateStatus()
    }

    private fun updateStatus() {
        val scanText = if (scanning) "Escaneando" else "No escaneando"
        val sendText = if (sending) "Enviando UDP" else "No enviando UDP"
        tvStatus.text = "Estado: $scanText / $sendText"
        btnScan.text = if (scanning) "Detener Scan BLE" else "Scan BLE"
        btnSend.text = if (sending) "Detener Send BLE" else "Send BLE"
    }

    private fun hasPermissions(): Boolean {
        val fineLocation = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        val btScan = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED else true
        val btConnect = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S)
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED else true
        return fineLocation && btScan && btConnect
    }

    private fun requestPermissions() {
        val perms = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            perms.add(Manifest.permission.BLUETOOTH_SCAN)
            perms.add(Manifest.permission.BLUETOOTH_CONNECT)
        }
        ActivityCompat.requestPermissions(this, perms.toTypedArray(), REQUEST_PERMISSION_CODE)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_PERMISSION_CODE) {
            if (grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
                Toast.makeText(this, "Permisos concedidos", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "Permisos denegados, la app no funcionará", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun startBleScan() {
        val bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled) {
            Toast.makeText(this, "Bluetooth no activado o disponible", Toast.LENGTH_LONG).show()
            return
        }
        bluetoothLeScanner = bluetoothAdapter.bluetoothLeScanner
        scanning = true
        deviceList.clear()
        adapter.notifyDataSetChanged()
        bluetoothLeScanner.startScan(scanCallback)
        updateStatus()
        Toast.makeText(this, "Escaneo BLE iniciado", Toast.LENGTH_SHORT).show()
    }

    private fun stopBleScan() {
        bluetoothLeScanner.stopScan(scanCallback)
        scanning = false
        updateStatus()
        Toast.makeText(this, "Escaneo BLE detenido", Toast.LENGTH_SHORT).show()
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val mac = result.device.address
            val rssi = result.rssi

            runOnUiThread {
                val index = deviceList.indexOfFirst { it.mac == mac }
                if (index >= 0) {
                    deviceList[index].rssi = rssi
                    adapter.notifyItemChanged(index)
                } else {
                    deviceList.add(Device(mac, rssi))
                    adapter.notifyItemInserted(deviceList.size - 1)
                }
            }

            if (sending) {
                thread {
                    sendUdp("$mac,$rssi\n")
                }
            }
        }

        override fun onScanFailed(errorCode: Int) {
            runOnUiThread {
                Toast.makeText(this@MainActivity, "Escaneo fallido: error $errorCode", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun sendUdp(message: String) {
        try {
            DatagramSocket().use { socket ->
                val ip = InetAddress.getByName(SERVER_IP)
                val buf = message.toByteArray()
                val packet = DatagramPacket(buf, buf.size, ip, SERVER_PORT)
                socket.send(packet)
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        if (scanning) {
            bluetoothLeScanner.stopScan(scanCallback)
        }
    }
}
