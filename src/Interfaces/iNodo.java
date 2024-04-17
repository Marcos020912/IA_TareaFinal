package Interfaces;

import java.util.List;

public interface iNodo<inf> {
    void agregarHijo(iNodo<inf> hijo);
    List<iNodo<inf>> getHijos();
    inf getDato();
}


