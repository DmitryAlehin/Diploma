using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Globalization;

namespace TCPClient
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent(); //конфигурация приложения
            Milliseconds.Interval = 1; 
            Milliseconds.Enabled = true; 
            //настройки масштабирования графика скоростей
            chart1.ChartAreas[0].CursorX.IsUserEnabled = true;
            chart1.ChartAreas[0].CursorX.IsUserSelectionEnabled = true;
            chart1.ChartAreas[0].AxisX.ScaleView.Zoomable = true;
            chart1.ChartAreas[0].AxisX.ScrollBar.IsPositionedInside = true;

            
            chart1.ChartAreas[0].CursorY.IsUserEnabled = true;
            chart1.ChartAreas[0].CursorY.IsUserSelectionEnabled = true;
            chart1.ChartAreas[0].AxisY.ScaleView.Zoomable = true;
            chart1.ChartAreas[0].AxisY.ScrollBar.IsPositionedInside = true;
            //настройки масштабирования графика углов
            chart2.ChartAreas[0].CursorX.IsUserEnabled = true;
            chart2.ChartAreas[0].CursorX.IsUserSelectionEnabled = true;
            chart2.ChartAreas[0].AxisX.ScaleView.Zoomable = true;
            chart2.ChartAreas[0].AxisX.ScrollBar.IsPositionedInside = true;
            
            chart2.ChartAreas[0].CursorY.IsUserEnabled = true;
            chart2.ChartAreas[0].CursorY.IsUserSelectionEnabled = true;
            chart2.ChartAreas[0].AxisY.ScaleView.Zoomable = true;
            chart2.ChartAreas[0].AxisY.ScrollBar.IsPositionedInside = true;
            //настройки масштабирования графика высоты
            chart3.ChartAreas[0].CursorX.IsUserEnabled = true;
            chart3.ChartAreas[0].CursorX.IsUserSelectionEnabled = true;
            chart3.ChartAreas[0].AxisX.ScaleView.Zoomable = true;
            chart3.ChartAreas[0].AxisX.ScrollBar.IsPositionedInside = true;

            chart3.ChartAreas[0].CursorY.IsUserEnabled = true;
            chart3.ChartAreas[0].CursorY.IsUserSelectionEnabled = true;
            chart3.ChartAreas[0].AxisY.ScaleView.Zoomable = true;
            chart3.ChartAreas[0].AxisY.ScrollBar.IsPositionedInside = true;
        }
        int TimerCount = 0;
        private static Socket client;
        private static byte[] data = new byte[1024];
        
        //описание функции нажатия кнопки "Соединить"
        private void connect_Click(object sender, EventArgs e)
        {
            results.Items.Add("Connecting...");
            client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            IPEndPoint iep = new IPEndPoint(IPAddress.Parse("192.168.4.1"), 88);
            client.BeginConnect(iep, new AsyncCallback(Connected), client);
        }        
        void Connected(IAsyncResult iar)
        {
            try
            {
                client.EndConnect(iar);
                results.Items.Add("Connected to:" + client.RemoteEndPoint.ToString());
                Thread receiver = new Thread(new ThreadStart(ReceiveData));
                receiver.Start();
            }
            catch (SocketException)
            {
                results.Items.Add("Error Connecting");
            }
        }
        //описание функции приема данных
        void ReceiveData()
        {
            //смена , на . при приеме данных, чтобы программа видела числа с плавающей точкой
            IFormatProvider formatter = new NumberFormatInfo { NumberDecimalSeparator = "." };
            //объявление переменных
            int recv;
            string StringData, s1, s2, s3, s4, s5, s6, s7;
            float Altitude, SetSpeed, CurSpeed, Pitch, Roll, Yaw;
            while (true)
            {
                //прием данных
                recv = client.Receive(data);                
                StringData = Encoding.ASCII.GetString(data, 0, recv);
                //разделение полученного сообщения на несколько массивов по признаку пробела
                string[] k = StringData.Split(' ');
                //получение численного значения высоты
                Altitude = float.Parse(k[0], formatter);
                //получение численного значения заданной скорости
                SetSpeed = float.Parse(k[1], formatter);
                //получение численного значения текущей скорости
                CurSpeed = float.Parse(k[2], formatter);
                //получение численного значения тангажа
                Pitch = float.Parse(k[3], formatter);
                //получение численного значения крена
                Roll = float.Parse(k[4], formatter);
                //получение численного значения курса
                Yaw = float.Parse(k[5], formatter);
                s1 = Altitude.ToString();
                s2 = SetSpeed.ToString();
                s3 = CurSpeed.ToString();
                s4 = Pitch.ToString();
                s5 = Roll.ToString();
                s6 = Yaw.ToString();
                //показ полученных значений на главной странице
                results.Items.Add("Высота: " + s1 + " Vзад: " + s2 + " Vтек: " + s3 + " Тангаж: " + s4 + " Крен:" + s5 + " Курс:" + s6);

                //построение графика скоростей
                chart1.Series[0].Points.AddXY(TimerCount, SetSpeed);
                chart1.Series[1].Points.AddXY(TimerCount, CurSpeed);
                chart1.ChartAreas[0].AxisX.ScaleView.Zoom(TimerCount-1000, TimerCount);
                //построение графика углов
                chart2.Series[0].Points.AddXY(TimerCount, Pitch);
                chart2.Series[1].Points.AddXY(TimerCount, Roll);
                chart2.Series[2].Points.AddXY(TimerCount, Yaw);
                chart2.ChartAreas[0].AxisX.ScaleView.Zoom(TimerCount-1000, TimerCount);
                //построение графика высоты
                chart3.Series[0].Points.AddXY(TimerCount, Altitude);
                chart3.ChartAreas[0].AxisX.ScaleView.Zoom(TimerCount - 1000, TimerCount);
                results.TopIndex = results.Items.Count - 1;
            }          
        }

        //описание функции нажатия на кнопку "Отключение"
        private void exit_Click(object sender, EventArgs e)
        {
            client.Close();
            results.Items.Add("Connection stopped");
            Close();
        }

        //описание функции обновления дисплея
        private delegate void DlDisplay(string s);
        private void Display(string s)
        {
            if (results.InvokeRequired)
            {
                DlDisplay sd = new DlDisplay(Display);
                results.Invoke(sd, new object[] { s });                
            }
            else
            {
                results.Items.Add(s);                
            }
        }

        //описание функции нажатия на кнопку "Отправить"
        private void sendit_Click(object sender, EventArgs e)
        {
            byte[] message = Encoding.ASCII.GetBytes(newText.Text);
            newText.Clear();
            client.BeginSend(message, 0, message.Length, 0, new AsyncCallback(SendData), client);
        }

        //функция отправки данных
        void SendData(IAsyncResult iar)
        {
            Socket remote = (Socket)iar.AsyncState;
            int sent = remote.EndSend(iar);
        }

        //функция подстчета количества тактов для получения времени
        private void timer1_Tick(object sender, EventArgs e)
        {
            TimerCount += 1;            
        }

        //функция отмены масштабирования при двойном нажатии правой кнопки мыши для графика скоростей
        private void chart1_DoubleClick(object sender, EventArgs e)
        {
            chart1.ChartAreas[0].AxisX.ScaleView.ZoomReset(0);
            chart1.ChartAreas[0].AxisY.ScaleView.ZoomReset(0);
        }

        //функция отмены масштабирования при двойном нажатии правой кнопки мыши для графика углов
        private void chart2_DoubleClick(object sender, EventArgs e)
        {
            chart2.ChartAreas[0].AxisX.ScaleView.ZoomReset(0);
            chart2.ChartAreas[0].AxisY.ScaleView.ZoomReset(0);
        }

        //функция отмены масштабирования при двойном нажатии правой кнопки мыши для графика высоты
        private void chart3_DoubleClick(object sender, EventArgs e)
        {
            chart3.ChartAreas[0].AxisX.ScaleView.ZoomReset(0);
            chart3.ChartAreas[0].AxisY.ScaleView.ZoomReset(0);
        }

        
    }

    
}


