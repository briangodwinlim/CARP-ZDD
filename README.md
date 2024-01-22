# Capacitated Arc Routing Problem with Zero-Suppressed Binary Decision Diagram

A novel optimization algorithm for an exact solution to the capacitated arc routing problem using zero-suppressed binary decision diagrams.

## Usage

### Build

```
make
```

### CARP-ZDD

```
./carp <graph_file> [ <option>... ]
```

#### Example

```
./carp Graphs/kshs1.dat -solution
```

### Graph File Format

The `graph_file` follows the format described [here](https://www.uv.es/~belengue/carp/READ_ME). The graph files may be downloaded [here](https://www.uv.es/belengue/carp.html).

## Related Repositories

- [TdZdd Library](https://github.com/kunisura/TdZdd/)