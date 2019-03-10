SUPER  NESPI CASE
=================


Requirements
------------
- `Raspberry Pi <https://www.raspberrypi.org/>`_
- `Super NESPi Case <http://snespi.com/>`_ (Important, **not** `Retroflag Case <http://retroflag.com/SUPERPi-CASE-J.html>`_!)
- Python 2.7
- `RPi.GPIO <https://sourceforge.net/projects/raspberry-gpio-python/>`_



Installation
------------
All included scripts requires ``settings.ini`` file to be present in ``/home/pi/snespi/`` directory.

Temperature Fan Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~
1) Copy file ``fan_controller.py`` to ``/home/pi/snespi/`` directory
2) Modify ``settings.ini`` if needed (Warning! Setting incorrect settings may lead to board damage!)
3) Configure script autostart
    a) Edit ``rc.local`` file.
    b) Insert following code before ``exit 0`` (``exit 0`` must be a last line in file):
        .. code-block:: bash

            /usr/bin/python2.7 /home/pi/snespi/fan_controller.py
    c) Reboot: ``sudo reboot``


License
-------
MIT


Creator
-------
Daniel Kuruc
