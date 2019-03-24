SUPER  NESPI CASE
=================

Python GPIO Tools for `Super NESPi Case <http://snespi.com/>`_.

.. image:: snespi.jpg

Tools Included
--------------
1. Temperature Fan Controller
2. Soft Reset Button
3. (Work In Progress) Soft Shutdown Button

Requirements
------------

Common
~~~~~~
- `Raspberry Pi <https://www.raspberrypi.org/>`_
- `Super NESPi Case <http://snespi.com/>`_ (Important, **not** `Retroflag Case <http://retroflag.com/SUPERPi-CASE-J.html>`_!)
- Python 2.7
- `RPi.GPIO <https://sourceforge.net/projects/raspberry-gpio-python/>`_

Temperature Fan Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~
- `mock <https://pypi.org/project/mock/>`_ Python package (for simulation mode)
- `simple-pid <https://pypi.org/project/simple-pid/>`_ Python package


Installation
------------
1. Install required Python packages::

      pip install -r requirements.txt
2. Copy file ``_snespi.py`` file to ``/home/pi/snespi/`` directory.
3. Copy file ``settings.ini`` file to ``/home/pi/snespi/`` directory.

   | You can override default settings, by creating file ``settings.local.ini``, next to ``settings.ini`` and selectively placing there your override values (BE CAREFUL! Setting incorrect settings may lead to board damage!)


Temperature Fan Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~
1. Copy file following files to ``/home/pi/snespi/`` directory on your Raspberry Pi:

   - ``fan_controller.py``
   - ``settings.ini``
2. Configure script autostart

   a. Run::

         sudo crontab -e -u root
   b. Add new entry to crontab configuraion::

         @reboot /usr/bin/python2.7 /home/pi/snespi/fan_controller.py & > /dev/null 2>&1
   c. Save file & close editor
   d. Reboot your RPi: ``sudo reboot``

Soft Reset Button
~~~~~~~~~~~~~~~~~
1. Copy file following files to ``/home/pi/snespi/`` directory on your Raspberry Pi:

   - ``soft_reset.py``
   - ``settings.ini``
2. Configure script autostart

   a. Run::

         sudo crontab -e -u root
   b. Add new entry to crontab configuraion::

         @reboot /usr/bin/python2.7 /home/pi/snespi/soft_reset.py & > /dev/null 2>&1
   c. Save file & close editor
   d. Reboot your RPi: ``sudo reboot``


License
-------
`MIT <LICENSE>`_


Creator
-------
Daniel Kuruc
