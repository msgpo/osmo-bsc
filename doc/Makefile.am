SUBDIRS = \
	examples \
	old \
	$(NULL)

msc: \
	$(builddir)/handover.png \
	$(builddir)/assignment.png \
	$(builddir)/lchan-release.png \
	$(builddir)/ms-channel-request.png \
	$(builddir)/timeslot.png \
	$(builddir)/lchan.png \
	$(builddir)/ts-and-lchan-fsm-lifecycle.png \
	$(builddir)/handover-inter-bsc-mo.png \
	$(builddir)/handover-inter-bsc-mt.png \
	$(NULL)

dot: \
	$(builddir)/assignment-fsm.png \
	$(builddir)/timeslot-fsm.png \
	$(builddir)/lchan-fsm.png \
	$(builddir)/mgw-endpoint-fsm.png \
	$(builddir)/handover-intra-bsc-fsm.png \
	$(builddir)/handover-inter-bsc-mo-fsm.png \
	$(builddir)/handover-inter-bsc-mt-fsm.png \
	$(NULL)

$(builddir)/%.png: $(srcdir)/%.msc
	mscgen -T png -o $@ $<

$(builddir)/%.png: $(srcdir)/%.dot
	dot -Tpng $< > $@

.PHONY: poll
poll:
	while true; do $(MAKE) msc dot; sleep 1; done
