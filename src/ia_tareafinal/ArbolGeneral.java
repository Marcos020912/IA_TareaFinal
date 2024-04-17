package ia_tareafinal;

import Interfaces.iNodo;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class ArbolGeneral<inf> {

    // Nodo como clase interna de ArbolGeneral
    private class Nodo implements iNodo<inf> {

        private inf dato;
        private List<iNodo<inf>> hijos;

        public Nodo(inf dato) {
            this.dato = dato;
            this.hijos = new ArrayList<>();
        }

        @Override
        public void agregarHijo(iNodo<inf> hijo) {
            this.hijos.add(hijo);
        }

        @Override
        public List<iNodo<inf>> getHijos() {
            return hijos;
        }

        @Override
        public inf getDato() {
            return dato;
        }
    }

    private iNodo<inf> raiz;

    public ArbolGeneral(inf datoRaiz) {
        this.raiz = new Nodo(datoRaiz);
    }

    public void agregarHijo(iNodo<inf> padre, iNodo<inf> hijo) {
        padre.agregarHijo(hijo);
    }

    public Iterator<iNodo<inf>> buscarCamino(iNodo<inf> inicio, iNodo<inf> fin) {
        List<iNodo<inf>> camino = new ArrayList<>();
        buscarCaminoDFS(inicio, fin, camino);
        return camino.iterator();
    }

    private boolean buscarCaminoDFS(iNodo<inf> actual, iNodo<inf> fin, List<iNodo<inf>> camino) {
        if (actual == null) {
            return false;
        }
        camino.add(actual); // Añade el nodo actual al camino

        if (actual.equals(fin)) {
            return true; // Encontró el nodo final
        }

        for (iNodo<inf> hijo : actual.getHijos()) {
            if (buscarCaminoDFS(hijo, fin, camino)) {
                return true; // Encontró el camino en uno de los hijos
            }
        }

        camino.remove(camino.size() - 1); // Elimina el nodo actual del camino si no lleva al final
        return false;
    }
    
    public Iterator<iNodo<inf>> buscarCaminoBFS(iNodo<inf> inicio, iNodo<inf> finalObjetivo) {
        if (inicio == null || finalObjetivo == null) return Collections.emptyIterator();

        Queue<List<iNodo<inf>>> cola = new LinkedList<>();
        List<iNodo<inf>> caminoInicial = new ArrayList<>();
        caminoInicial.add(inicio);
        cola.add(caminoInicial);

        while (!cola.isEmpty()) {
            List<iNodo<inf>> caminoActual = cola.poll();
            iNodo<inf> nodoActual = caminoActual.get(caminoActual.size() - 1);

            if (nodoActual.equals(finalObjetivo)) {
                return caminoActual.iterator();
            }

            for (iNodo<inf> hijo : nodoActual.getHijos()) {
                List<iNodo<inf>> nuevoCamino = new ArrayList<>(caminoActual);
                nuevoCamino.add(hijo);
                cola.add(nuevoCamino);
            }
        }

        return Collections.emptyIterator(); // No se encontró un camino
    }
}
